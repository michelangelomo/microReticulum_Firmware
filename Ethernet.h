// RAK13800 W5100S Ethernet bring-up for RAK4631 in RAK19007 IO_SLOT.
// Single-include header: declares globals and provides ethernet_init() /
// update_ethernet(). Include from Utilities.h under HAS_ETHERNET == true,
// mirroring Remote.h.

#if HAS_ETHERNET == true

#include <SPI.h>
#include <RAK13800_W5100S.h>

#define ETH_UPDATE_INTERVAL_MS 500
#define ETH_DHCP_TIMEOUT_MS    15000
#define ETH_DHCP_RESP_MS       4000

// EEPROM read for NRF52 is defined later in Utilities.h; forward-declare so
// this single-include header can use it without reordering.
#if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
uint8_t eeprom_read(uint32_t mapped_addr);
#endif

bool ethernet_enabled = false;      // User opt-in via ADDR_ETH_ENABLE
bool ethernet_initialized = false;  // Link is up / IP acquired
uint8_t eth_mac[6];
IPAddress eth_device_ip;
uint32_t last_eth_update = 0;

char eth_host[ETH_HOST_MAXLEN + 1];
uint16_t eth_port = 0;

#if defined(TCP_TRANSPORT)
EthernetClient tcp_client;
#if defined(HAS_RNS)
RNS::Interface tcp_interface(RNS::Type::NONE);
#endif
#endif

// Derive a locally-administered unicast MAC from the nRF52 factory device
// address (NRF_FICR->DEVICEADDR). Stable per chip, unique across units.
void ethernet_derive_mac(uint8_t mac_out[6]) {
  uint32_t a0 = NRF_FICR->DEVICEADDR[0];
  uint32_t a1 = NRF_FICR->DEVICEADDR[1];
  mac_out[0] = ((a1 >>  8) & 0xFC) | 0x02; // locally-administered, unicast
  mac_out[1] = ( a1        & 0xFF);
  mac_out[2] = ((a0 >> 24) & 0xFF);
  mac_out[3] = ((a0 >> 16) & 0xFF);
  mac_out[4] = ((a0 >>  8) & 0xFF);
  mac_out[5] = ( a0        & 0xFF);
}

void ethernet_power_on() {
  pinMode(pin_eth_en, OUTPUT);
  digitalWrite(pin_eth_en, HIGH);
  pinMode(pin_eth_rst, OUTPUT);
  digitalWrite(pin_eth_rst, LOW);
  delay(200);
  digitalWrite(pin_eth_rst, HIGH);
  delay(500);
}

static inline uint8_t eth_cfg_byte(uint32_t addr) {
#if HAS_EEPROM
  return EEPROM.read(addr);
#else
  return eeprom_read(addr);
#endif
}

// Returns true if stored IPv4 is non-zero and non-0xFF (i.e. configured).
static bool ethernet_read_ipv4(uint32_t base, IPAddress& out) {
  uint8_t b[4];
  bool any_set = false, all_ff = true;
  for (uint8_t i = 0; i < 4; i++) {
    b[i] = eth_cfg_byte(base + i);
    if (b[i] != 0x00) any_set = true;
    if (b[i] != 0xFF) all_ff  = false;
  }
  if (!any_set || all_ff) return false;
  out = IPAddress(b[0], b[1], b[2], b[3]);
  return true;
}

static inline bool eth_is_host_char(uint8_t c) {
  return (c >= 'a' && c <= 'z') ||
         (c >= 'A' && c <= 'Z') ||
         (c >= '0' && c <= '9') ||
          c == '.' || c == '-';
}

// Returns true only if the stored host looks like a real DNS/IPv4 string
// terminated by 0x00/0xFF. Any non-hostname byte — i.e. uninitialized garbage
// left over in the pre-offset EEPROM region — rejects the whole field.
static bool ethernet_load_host(char* out, size_t cap) {
  size_t n = 0;
  for (size_t i = 0; i < ETH_HOST_MAXLEN && n + 1 < cap; i++) {
    uint8_t c = eth_cfg_byte(ADDR_ETH_HOST + i);
    if (c == 0x00 || c == 0xFF) break;
    if (!eth_is_host_char(c)) { out[0] = '\0'; return false; }
    out[n++] = (char)c;
  }
  out[n] = '\0';
  return n > 0;
}

void ethernet_load_config() {
  bool host_ok = ethernet_load_host(eth_host, sizeof(eth_host));
  // Port is only meaningful alongside a valid host. Otherwise the bytes are
  // indistinguishable from uninitialized garbage.
  if (host_ok) {
    uint8_t ph = eth_cfg_byte(ADDR_ETH_PORT);
    uint8_t pl = eth_cfg_byte(ADDR_ETH_PORT + 1);
    if (ph == 0xFF && pl == 0xFF) eth_port = 0;
    else                          eth_port = ((uint16_t)ph << 8) | pl;
  } else {
    eth_port = 0;
  }
}

void ethernet_init() {
  // Always power on the module so it can be detected, regardless of enable byte.
  ethernet_power_on();
  ethernet_derive_mac(eth_mac);
  Ethernet.init(pin_eth_cs);

  // Use a dummy static IP to trigger SPI initialisation inside the W5100 library
  // without blocking on DHCP. On nRF52, hardwareStatus() before begin() does not
  // fully initialise SPI — begin() must be called first.
  IPAddress dummy(169, 254, 1, 1);
  Ethernet.begin(eth_mac, dummy, dummy, dummy, IPAddress(255, 255, 0, 0));

  // Hardware presence check — bail immediately if W5100S not responding.
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("[ETH] RAK13800 not detected — check module seating.");
    ethernet_initialized = false;
    return;
  }
  Serial.println("[ETH] RAK13800 detected.");
  ethernet_enabled = true;

  ethernet_load_config();

  uint8_t dhcp_byte = eth_cfg_byte(ADDR_ETH_DHCP);
  bool use_dhcp = (dhcp_byte != 0x00);

  if (use_dhcp) {
    Serial.println("[ETH] Requesting DHCP lease...");
    if (Ethernet.begin(eth_mac, ETH_DHCP_TIMEOUT_MS, ETH_DHCP_RESP_MS) == 0) {
      // DHCP failed. If a static IP is stored, fall back to it.
      IPAddress ip, nm, gw, dns;
      bool have_ip = ethernet_read_ipv4(ADDR_ETH_IP, ip);
      if (!have_ip) { ethernet_initialized = false; return; }
      bool have_nm  = ethernet_read_ipv4(ADDR_ETH_NM,  nm);
      bool have_gw  = ethernet_read_ipv4(ADDR_ETH_GW,  gw);
      bool have_dns = ethernet_read_ipv4(ADDR_ETH_DNS, dns);
      if (have_nm && have_gw && have_dns) {
        Ethernet.begin(eth_mac, ip, dns, gw, nm);
      } else if (have_nm && have_gw) {
        Ethernet.begin(eth_mac, ip, IPAddress(1,1,1,1), gw, nm);
      } else {
        Ethernet.begin(eth_mac, ip);
      }
    }
  } else {
    IPAddress ip, nm, gw, dns;
    if (!ethernet_read_ipv4(ADDR_ETH_IP, ip)) { ethernet_initialized = false; return; }
    bool have_nm  = ethernet_read_ipv4(ADDR_ETH_NM,  nm);
    bool have_gw  = ethernet_read_ipv4(ADDR_ETH_GW,  gw);
    bool have_dns = ethernet_read_ipv4(ADDR_ETH_DNS, dns);
    if (have_nm && have_gw && have_dns) {
      Ethernet.begin(eth_mac, ip, dns, gw, nm);
    } else if (have_nm && have_gw) {
      Ethernet.begin(eth_mac, ip, IPAddress(1,1,1,1), gw, nm);
    } else {
      Ethernet.begin(eth_mac, ip);
    }
  }

  eth_device_ip = Ethernet.localIP();
  ethernet_initialized = true;
  Serial.print("[ETH] IP address: ");
  Serial.println(eth_device_ip);
}

void update_ethernet() {
  if (!ethernet_initialized) return;
  if (millis() - last_eth_update >= ETH_UPDATE_INTERVAL_MS) {
    Ethernet.maintain();
    last_eth_update = millis();
  }
}

#endif
