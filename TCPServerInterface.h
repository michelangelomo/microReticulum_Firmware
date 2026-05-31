#include <Reticulum.h>
#include <Interface.h>
#include <Log.h>
#include <Bytes.h>

#include <RAK13800_W5100S.h>

#define TCP_SERVER_DEFAULT_PORT 4242
#define TCP_SERVER_RX_BUF_SIZE  2048

extern bool ethernet_initialized;

#if defined(HAS_RNS) && defined(TCP_TRANSPORT)

// Reticulum TCP server interface with HDLC framing (0x7E flag, 0x7D escape,
// 0x20 mask). Listens on a LAN port; one active peer at a time (W5100S has
// 4 hardware sockets — one is shared with the client interface when present).
// Allows host PCs running rnsd to peer in directly over Ethernet, replacing
// USB as the local access method (Reticulum manual §8.4 TCP Server Interface).
class TCPServerInterface : public RNS::InterfaceImpl {
  static constexpr uint8_t HDLC_FLAG     = 0x7E;
  static constexpr uint8_t HDLC_ESC      = 0x7D;
  static constexpr uint8_t HDLC_ESC_MASK = 0x20;

public:
  explicit TCPServerInterface(uint16_t port = TCP_SERVER_DEFAULT_PORT)
      : RNS::InterfaceImpl("TCPServerInterface"), _server(port), _port(port) {
    _IN      = true;
    _OUT     = true;
    _FWD     = true;
    _HW_MTU  = 1064;
    _bitrate = 10000000; // 10 Mbps (W5100S rated speed)
  }
  virtual ~TCPServerInterface() { _name = "deleted"; }

protected:
  virtual bool start() override {
    if (!ethernet_initialized) return false;
    _server.begin();
    INFOF("TCPServerInterface: listening on port %u", (unsigned)_port);
    return true;
  }

  virtual void stop() override {
    if (_active_client) {
      _active_client.stop();
    }
    _in_frame = false;
    _escape   = false;
    _rx_len   = 0;
  }

  virtual void loop() override {
    if (!ethernet_initialized) return;

    // Accept new connection only when no active peer.
    if (!_active_client || !_active_client.connected()) {
      if (_active_client) {
        INFO("TCPServerInterface: peer disconnected");
        _active_client.stop();
        _in_frame = false;
        _escape   = false;
        _rx_len   = 0;
      }
      EthernetClient candidate = _server.available();
      if (candidate) {
        _active_client = candidate;
        INFO("TCPServerInterface: peer connected");
      }
      return;
    }

    // Drain RX from active peer.
    int avail = _active_client.available();
    while (avail-- > 0) {
      int b = _active_client.read();
      if (b < 0) break;
      _feed_byte((uint8_t)b);
    }
  }

  virtual bool send_outgoing(const RNS::Bytes& data) override {
    try {
      if (ethernet_initialized && _active_client && _active_client.connected()) {
        TRACEF("TCPServerInterface.send_outgoing: (%u bytes)", data.size());
        _active_client.write(HDLC_FLAG);
        const uint8_t* buf = data.data();
        size_t n = data.size();
        for (size_t i = 0; i < n; i++) {
          uint8_t c = buf[i];
          if (c == HDLC_FLAG || c == HDLC_ESC) {
            uint8_t esc[2] = { HDLC_ESC, (uint8_t)(c ^ HDLC_ESC_MASK) };
            _active_client.write(esc, 2);
          } else {
            _active_client.write(c);
          }
        }
        _active_client.write(HDLC_FLAG);
      }
      InterfaceImpl::handle_outgoing(data);
    }
    catch (const std::bad_alloc&) {
      ERROR("TCPServerInterface::send_outgoing: bad_alloc - out of memory");
      return false;
    }
    catch (std::exception& e) {
      ERRORF("TCPServerInterface::send_outgoing: %s", e.what());
      return false;
    }
    return true;
  }

private:
  void _feed_byte(uint8_t b) {
    if (b == HDLC_FLAG) {
      if (_in_frame && _rx_len > 0) {
        try {
          RNS::Bytes frame(_rx_buf, _rx_len);
          TRACEF("TCPServerInterface.handle_incoming: (%u bytes)", (unsigned)_rx_len);
          InterfaceImpl::handle_incoming(frame);
        }
        catch (const std::bad_alloc&) {
          ERROR("TCPServerInterface::handle_incoming: bad_alloc - out of memory");
        }
        catch (std::exception& e) {
          ERRORF("TCPServerInterface::handle_incoming: %s", e.what());
        }
      }
      _in_frame = true;
      _rx_len   = 0;
      _escape   = false;
      return;
    }
    if (!_in_frame) return;
    if (_escape) {
      b ^= HDLC_ESC_MASK;
      _escape = false;
    } else if (b == HDLC_ESC) {
      _escape = true;
      return;
    }
    if (_rx_len < TCP_SERVER_RX_BUF_SIZE) {
      _rx_buf[_rx_len++] = b;
    } else {
      // Oversize frame — drop and resync on next FLAG.
      _in_frame = false;
      _rx_len   = 0;
      _escape   = false;
    }
  }

private:
  EthernetServer _server;
  EthernetClient _active_client;
  uint16_t       _port;
  uint8_t        _rx_buf[TCP_SERVER_RX_BUF_SIZE];
  size_t         _rx_len   = 0;
  bool           _in_frame = false;
  bool           _escape   = false;
};

#endif
