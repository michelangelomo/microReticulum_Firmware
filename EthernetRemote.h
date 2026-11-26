// Copyright (C) 2024, Mark Qvist

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// KISS-over-TCP host transport on a WIZnet W5100S (RAK13800 WisBlock
// Ethernet module). Mirrors the WiFi remote in Remote.h: one host at a
// time on TCP port 7633, idle connections are dropped, and losing the host
// stops the radio via host_disconnected().
//
// Bus layout on the RAK4631: the W5100S sits on the board-default SPI
// (SPIM3, IO-slot pins) behind the shared IO-slot chip select, while the
// SX1262 uses spiModem (SPIM2) on the core-slot pins. The two buses never
// contend, so the modem ISR is unaffected by Ethernet traffic.
//
// The module is detected at runtime so a single firmware image serves
// boards with and without it. Every W5100S library call that goes through
// W5100Class::init() re-runs the 560 ms chip probe until a chip has been
// found, so once the module is known to be absent nothing here touches the
// library again.
//
// Nothing in here may block the main loop, which also services the radio:
//
// - Output is queued in a local ring and sent with at most one SEND command
//   in flight. The library's write() spins until the chip reports SEND_OK
//   for every call, so writing each KISS frame through it stalled the loop
//   once per frame, and a frame was dropped whenever the chip's TX buffer
//   was momentarily short. Frames queued while a SEND is in flight go out
//   together in the next one; a frame is only dropped if the local ring
//   overflows, and then whole, so the KISS stream stays well-formed.
// - The listener is polled with accept() even while a host is attached. A
//   host that reconnects has given up on its previous connection, and the
//   chip accepts the new one in hardware either way, so the newest
//   connection replaces the old one instead of being left unserviced (the
//   host then reports "RNode detect timed out").
// - Closing a connection waits at most ETH_STOP_TIMEOUT_MS for the FIN
//   handshake instead of the library's default second.
//
// Every disconnect prints its reason on the serial console.

#include <RAK13800_W5100S.h>
#include <w5100.h>   // W5100 register access for the non-blocking send path
#include <SPI.h>

#define ETH_TCP_PORT           7633
#define ETH_READ_TIMEOUT_MS    6500   // same idle window as the WiFi remote (WR_READ_TIMEOUT_MS)
#define ETH_DHCP_TIMEOUT_MS    3000   // Ethernet.begin() blocks the main loop for at most this long
#define ETH_DHCP_RESPONSE_MS   1500
#define ETH_RETRY_MS           30000  // between DHCP attempts while the link is down or unconfigured
#define ETH_MAINTAIN_MS        1000   // DHCP lease bookkeeping cadence
#define ETH_CHIP_CHECK_MS      5000   // brownout detection cadence
#define ETH_ACCEPT_POLL_MS     20     // listener poll cadence, also while a host is attached
#define ETH_RX_POLL_MS         2      // socket poll cadence while the local RX buffer is empty
#define ETH_TX_FLUSH_MS        20     // send output that is not FEND-terminated after this long
#define ETH_STOP_TIMEOUT_MS    20     // FIN handshake wait when closing a connection
#define ETH_RX_BUF_SIZE        512
#define ETH_TX_RING_SIZE       4096   // several fully escaped maximum-length KISS frames
#define ETH_RAIL_SETTLE_MS     100    // 3V3_S switch-on to first reset pulse
#define ETH_PROBE_ATTEMPTS     3      // failed chip probes before the module is declared absent

uint32_t eth_last_read        = 0;
uint32_t eth_last_poll        = 0;
uint32_t eth_last_accept      = 0;
uint32_t eth_last_retry       = 0;
uint32_t eth_last_maintain    = 0;
uint32_t eth_last_chip_check  = 0;
bool     eth_initialized      = false;
bool     eth_hw_detected      = false; // W5100S answered at least once
bool     eth_hw_absent        = false; // W5100S confirmed absent; never touch the library again
uint8_t  eth_probe_failures   = 0;

// Bytes read from the socket in bulk and handed out one at a time by
// eth_remote_read(). Refilled only when empty, so a plain linear buffer
// is enough.
static uint8_t  eth_rx_buf[ETH_RX_BUF_SIZE];
static uint16_t eth_rx_pos = 0;
static uint16_t eth_rx_len = 0;

// Outbound ring. Bytes up to eth_tx_committed belong to complete KISS frames
// and may be sent; the bytes after it are the frame still being written.
static uint8_t  eth_tx_ring[ETH_TX_RING_SIZE];
static uint16_t eth_tx_head      = 0;     // next byte to send
static uint16_t eth_tx_len       = 0;     // queued bytes, committed or not
static uint16_t eth_tx_committed = 0;     // queued bytes that may be sent
static bool     eth_tx_in_frame  = false; // between an opening and a closing FEND
static bool     eth_tx_dropping  = false; // current frame did not fit; discard it to its end
static bool     eth_tx_inflight  = false; // a SEND command has not reported SEND_OK yet
static uint32_t eth_tx_last      = 0;
static uint32_t eth_tx_dropped_frames = 0;

EthernetServer eth_listener(ETH_TCP_PORT);
EthernetClient eth_connection;

uint8_t eth_mac[6];

extern void host_disconnected();
extern volatile uint8_t queue_height;  // pending TX packets, RNode_Firmware.ino
extern bool spi_ss_in_use;             // shared IO-slot CS already claimed by an external flash

static void eth_get_mac(uint8_t *mac) {
  // Derive a stable, unique MAC from the nRF52840 FICR device address
  // (the same 64-bit value the BLE stack uses for its public address).
  uint32_t addr0 = NRF_FICR->DEVICEADDR[0];
  uint32_t addr1 = NRF_FICR->DEVICEADDR[1];
  mac[0] = (addr1 >>  8) & 0xFF;
  mac[1] =  addr1        & 0xFF;
  mac[2] = (addr0 >> 24) & 0xFF;
  mac[3] = (addr0 >> 16) & 0xFF;
  mac[4] = (addr0 >>  8) & 0xFF;
  mac[5] =  addr0        & 0xFF;
  mac[0] &= 0xFE; // unicast
  mac[0] |= 0x02; // locally administered
}

// Radio-quiet gap: no carrier and nothing queued for transmission. The
// blocking library calls (DHCP, chip re-init) are only allowed here so they
// cannot interrupt a packet in flight.
static bool eth_radio_quiet() { return !dcd && queue_height == 0; }

bool eth_host_is_connected() { return (bool)eth_connection; }

// True while bytes read from the socket are still waiting in the RX buffer.
bool eth_remote_pending() { return eth_rx_len > 0; }

static void eth_tx_reset() {
  eth_tx_head = eth_tx_len = eth_tx_committed = 0;
  eth_tx_in_frame = eth_tx_dropping = eth_tx_inflight = false;
}

static void eth_close(EthernetClient &client) {
  client.setConnectionTimeout(ETH_STOP_TIMEOUT_MS);
  client.stop();
}

// Connections still waiting in the listener are left alone: one may be the
// host reconnecting, and eth_poll_listener() adopts it on its next pass.
static void eth_close_all() {
  eth_tx_reset();
  eth_rx_pos = 0;
  eth_rx_len = 0;
  if (eth_connection) { eth_close(eth_connection); }
}

// Console output is routed to the Ethernet host while one is attached, so the
// reason is printed after closing; otherwise it would go to the host being
// dropped instead of the serial console.
static void eth_disconnect(const char *reason) {
  eth_close_all();
  printf("[ETH] host disconnected: %s (%lu frames dropped)\n", reason, (unsigned long)eth_tx_dropped_frames);
  host_disconnected();
}

// Socket states in which data can still be exchanged with the host.
static bool eth_socket_usable(uint8_t status) {
  return status == SnSR::ESTABLISHED || status == SnSR::CLOSE_WAIT;
}

// Copy n ring bytes starting at eth_tx_head into the socket TX buffer at its
// write pointer, the same way the library's write_data() does. The caller
// holds the SPI transaction.
static void eth_tx_copy(uint8_t s, uint16_t n) {
  uint16_t ptr = W5100.readSnTX_WR(s);
  while (n > 0) {
    uint16_t chunk = ETH_TX_RING_SIZE - eth_tx_head;   // contiguous in the ring
    if (chunk > n) { chunk = n; }
    uint16_t offset = ptr & W5100.SMASK;
    uint16_t addr   = offset + W5100.SBASE(s);
    if (W5100.hasOffsetAddressMapping() || offset + chunk <= W5100.SSIZE) {
      W5100.write(addr, &eth_tx_ring[eth_tx_head], chunk);
    } else {
      uint16_t first = W5100.SSIZE - offset;             // wraps in the chip buffer
      W5100.write(addr, &eth_tx_ring[eth_tx_head], first);
      W5100.write(W5100.SBASE(s), &eth_tx_ring[eth_tx_head + first], chunk - first);
    }
    ptr             += chunk;
    eth_tx_head      = (eth_tx_head + chunk) % ETH_TX_RING_SIZE;
    eth_tx_len      -= chunk;
    eth_tx_committed -= chunk;
    n               -= chunk;
  }
  W5100.writeSnTX_WR(s, ptr);
}

// Move committed output to the chip without waiting for it: collect the
// SEND_OK of the previous SEND if it has arrived, then issue the next SEND.
static void eth_pump_tx() {
  if (!eth_connection || (eth_tx_committed == 0 && !eth_tx_inflight)) { return; }
  uint8_t s = eth_connection.getSocketNumber();

  W5100.getSPI()->beginTransaction(SPI_ETHERNET_SETTINGS);
  uint8_t status = W5100.readSnSR(s);
  if (!eth_socket_usable(status)) {
    W5100.getSPI()->endTransaction();
    char reason[40];
    snprintf(reason, sizeof(reason), "socket closed (status 0x%02X)", status);
    eth_disconnect(reason);
    return;
  }

  if (eth_tx_inflight) {
    uint8_t ir = W5100.readSnIR(s);
    if (ir & SnIR::SEND_OK) {
      W5100.writeSnIR(s, SnIR::SEND_OK);
      eth_tx_inflight = false;
    } else if (ir & SnIR::TIMEOUT) {
      W5100.writeSnIR(s, SnIR::TIMEOUT);
      W5100.getSPI()->endTransaction();
      eth_disconnect("TCP retransmission timeout");
      return;
    }
  }

  if (!eth_tx_inflight && eth_tx_committed > 0) {
    // TX_FSR can change while it is read; read until two reads agree.
    uint16_t free_size = W5100.readSnTX_FSR(s), prev;
    do { prev = free_size; free_size = W5100.readSnTX_FSR(s); } while (free_size != prev);
    uint16_t n = eth_tx_committed < free_size ? eth_tx_committed : free_size;
    if (n > 0) {
      eth_tx_copy(s, n);
      W5100.execCmdSn(s, Sock_SEND);
      eth_tx_inflight = true;
    }
  }
  W5100.getSPI()->endTransaction();
}

static void eth_tx_push(uint8_t byte) {
  uint16_t tail = (eth_tx_head + eth_tx_len) % ETH_TX_RING_SIZE;
  eth_tx_ring[tail] = byte;
  eth_tx_len++;
}

// Pull whatever the socket holds (up to the buffer size) in one transfer.
static bool eth_fill_rx() {
  int avail = eth_connection.available();
  if (avail <= 0) { return false; }
  if (avail > ETH_RX_BUF_SIZE) { avail = ETH_RX_BUF_SIZE; }
  int n = eth_connection.read(eth_rx_buf, avail);
  if (n <= 0) { return false; }
  eth_rx_pos    = 0;
  eth_rx_len    = n;
  eth_last_read = millis();
  return true;
}

// Adopt a newly accepted connection, replacing the current one if there is
// one. accept() hands out a connection as soon as it is established, so a
// host that has not sent anything yet is still serviced.
static void eth_poll_listener(uint32_t now) {
  if (now - eth_last_accept < ETH_ACCEPT_POLL_MS) { return; }
  eth_last_accept = now;
  EthernetClient client = eth_listener.accept();
  if (!client) { return; }
  if (eth_connection) {
    eth_close_all();
    printf("[ETH] new host connection replaces the current one\n");
    host_disconnected();      // the new host configures the radio from scratch
  }
  eth_connection = client;
  eth_last_read  = now;
  cable_state    = CABLE_STATE_CONNECTED;
}

bool eth_remote_available() {
  if (!eth_initialized) { return false; }
  if (eth_rx_len > 0)   { return true; }

  uint32_t now = millis();
  eth_poll_listener(now);
  if (!eth_connection) { return false; }

  if (now - eth_last_poll < ETH_RX_POLL_MS) { return false; }
  eth_last_poll = now;
  // connected() stays true for a half-closed socket that still holds
  // unread data, so the tail of a closing stream is not lost.
  if (!eth_connection.connected()) { eth_disconnect("connection closed"); return false; }
  if (eth_fill_rx()) { return true; }
  if (now - eth_last_read >= ETH_READ_TIMEOUT_MS) { eth_disconnect("idle timeout"); }
  return false;
}

uint8_t eth_remote_read() {
  if (eth_rx_len > 0) {
    uint8_t byte = eth_rx_buf[eth_rx_pos++];
    eth_rx_len--;
    return byte;
  }
  return FEND; // frame delimiter, harmless to the KISS parser
}

void eth_remote_write(uint8_t byte) {
  if (!eth_connection) { eth_tx_reset(); return; }
  eth_tx_last = millis();

  bool closing = byte == FEND && eth_tx_in_frame;
  bool opening = byte == FEND && !eth_tx_in_frame;
  if (opening) { eth_tx_in_frame = true; eth_tx_dropping = false; }

  if (!eth_tx_dropping) {
    if (eth_tx_len < ETH_TX_RING_SIZE) {
      eth_tx_push(byte);
    } else {
      // Ring full: discard the frame being written (its queued bytes and the
      // rest of it) and keep the complete frames before it.
      eth_tx_len      = eth_tx_committed;
      eth_tx_dropping = true;
      eth_tx_dropped_frames++;
    }
  }

  if (closing) {
    eth_tx_in_frame = false;
    if (!eth_tx_dropping) {
      eth_tx_committed = eth_tx_len;
      eth_pump_tx();
    }
    eth_tx_dropping = false;
  }
}

void eth_power_on() {
  pinMode(ETH_PWR_PIN, OUTPUT);
  digitalWrite(ETH_PWR_PIN, HIGH);
}

static void eth_hw_reset() {
  pinMode(ETH_RST_PIN, OUTPUT);
  digitalWrite(ETH_RST_PIN, LOW);  delay(100);
  digitalWrite(ETH_RST_PIN, HIGH); delay(100);
}

static bool eth_start() {
  if (eth_hw_absent) { return false; }
  Ethernet.init(SPI, ETH_CS_PIN);
  // Once the chip is known to exist, skip DHCP while the cable is out;
  // otherwise every retry would block for the full DHCP timeout.
  if (eth_hw_detected && Ethernet.linkStatus() != LinkON) { return false; }
  int status = Ethernet.begin(eth_mac, ETH_DHCP_TIMEOUT_MS, ETH_DHCP_RESPONSE_MS);
  if (status == 0) {
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      // A chip that was slow to come up must not be written off for good:
      // give it a few tries (30 s apart, from update_eth) before giving up.
      if (++eth_probe_failures >= ETH_PROBE_ATTEMPTS) { eth_hw_absent = true; }
    } else {
      eth_hw_detected = true;
    }
    return false;
  }
  eth_hw_detected = true;
  eth_listener.begin();
  IPAddress ip = Ethernet.localIP();
  printf("[ETH] link up, %u.%u.%u.%u:%u\n", ip[0], ip[1], ip[2], ip[3], ETH_TCP_PORT);
  return true;
}

// PoE power dips can reset the W5100S while the MCU keeps running, which
// reverts every register (including the MAC) to defaults. Re-run the whole
// bring-up when that happens.
static void eth_check_chip() {
  uint8_t current_mac[6];
  Ethernet.MACAddress(current_mac);
  if (memcmp(current_mac, eth_mac, 6) == 0) { return; }
  if (eth_connection) { eth_disconnect("W5100S reset"); }
  printf("[ETH] W5100S lost its configuration, reinitializing\n");
  eth_hw_reset();
  // The PHY needs time to negotiate after a reset and would report LinkOFF
  // too early; clearing the flag makes eth_start() skip the link gate once.
  eth_hw_detected = false;
  if (eth_start()) {
    eth_last_chip_check = millis();
  } else {
    eth_initialized = false;
    eth_last_retry  = millis();
  }
}

void update_eth() {
  if (eth_hw_absent) { return; }
  uint32_t now = millis();

  if (!eth_initialized) {
    if (now - eth_last_retry >= ETH_RETRY_MS && eth_radio_quiet()) {
      eth_last_retry = now;
      if (!eth_hw_detected) { eth_hw_reset(); } // still probing: start each attempt from a clean chip
      if (eth_start()) { eth_initialized = true; eth_last_chip_check = now; }
    }
    return;
  }

  // Output that never got a closing FEND is sent after a short pause.
  if (eth_tx_len > eth_tx_committed && now - eth_tx_last >= ETH_TX_FLUSH_MS) {
    eth_tx_committed = eth_tx_len;
    eth_tx_in_frame  = false;
    eth_tx_dropping  = false;
  }
  eth_pump_tx();

  if (eth_radio_quiet()) {
    // checkLease() counts elapsed time, so a renewal that falls due while
    // the radio is busy simply fires at the next quiet gap.
    if (now - eth_last_maintain >= ETH_MAINTAIN_MS) {
      eth_last_maintain = now;
      Ethernet.maintain();
    }
    if (now - eth_last_chip_check >= ETH_CHIP_CHECK_MS) {
      eth_last_chip_check = now;
      eth_check_chip();
    }
  }
}

// Called once at the very end of setup(). The RAK15001 probe earlier in
// setup() drives the same chip select, so the chip is reset here before
// first use. The rail itself is switched on at the top of setup(): on PoE
// the switch drops the port unless the board keeps drawing its minimum
// "maintain power signature" current, and only the W5100S gets a bare
// nRF52 with the radio off above that threshold.
bool eth_init() {
  if (spi_ss_in_use) {
    printf("[ETH] IO-slot chip select is used by external flash, Ethernet disabled\n");
    eth_hw_absent = true;
    return false;
  }

  eth_get_mac(eth_mac);
  eth_power_on(); // already on since the top of setup(); harmless to repeat
  delay(ETH_RAIL_SETTLE_MS);
  eth_hw_reset();

  if (!eth_start()) {
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      printf("[ETH] no W5100S module found, will probe again\n");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      printf("[ETH] cable not connected, will retry\n");
    } else {
      printf("[ETH] DHCP failed, will retry\n");
    }
    eth_last_retry = millis();
    return false;
  }

  eth_initialized     = true;
  eth_last_chip_check = millis();
  return true;
}
