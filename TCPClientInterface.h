#include <Reticulum.h>
#include <Interface.h>
#include <Log.h>
#include <Bytes.h>

#include <RAK13800_W5100S.h>

#define TCP_DEFAULT_HOST "amsterdam.connect.reticulum.network"
#define TCP_DEFAULT_PORT 4251
#define TCP_RECONNECT_INTERVAL_MS 10000
#define TCP_CONNECT_TIMEOUT_MS 8000
#define TCP_RX_BUF_SIZE 2048

extern EthernetClient tcp_client;
extern bool ethernet_initialized;

#if defined(HAS_RNS) && defined(TCP_TRANSPORT)

// Reticulum TCP client interface with HDLC framing (0x7E flag, 0x7D escape,
// 0x20 mask). Mirrors the Python reference's TCPClientInterface wire format.
class TCPClientInterface : public RNS::InterfaceImpl {
  static constexpr uint8_t HDLC_FLAG     = 0x7E;
  static constexpr uint8_t HDLC_ESC      = 0x7D;
  static constexpr uint8_t HDLC_ESC_MASK = 0x20;

public:
  TCPClientInterface(const char* name, const char* host, uint16_t port)
      : RNS::InterfaceImpl(name), _host(host), _port(port) {
    _IN  = true;
    _OUT = true;
    _FWD = true;
    _HW_MTU = 1064;
  }
  TCPClientInterface() : TCPClientInterface("TCPClientInterface", TCP_DEFAULT_HOST, TCP_DEFAULT_PORT) {}
  virtual ~TCPClientInterface() { _name = "deleted"; }

protected:
  virtual bool start() override {
    _attempt_connect();
    return true;
  }

  virtual void stop() override {
    tcp_client.stop();
    _connected = false;
    _in_frame  = false;
    _escape    = false;
    _rx_len    = 0;
  }

  virtual void loop() override {
    if (!ethernet_initialized) return;

    if (!tcp_client.connected()) {
      if (_connected) {
        INFO("TCPClientInterface: disconnected from peer");
        tcp_client.stop();
        _connected = false;
        _in_frame  = false;
        _escape    = false;
        _rx_len    = 0;
      }
      if (millis() - _last_reconnect >= TCP_RECONNECT_INTERVAL_MS) {
        _attempt_connect();
      }
      return;
    }

    int avail = tcp_client.available();
    while (avail-- > 0) {
      int b = tcp_client.read();
      if (b < 0) break;
      _feed_byte((uint8_t)b);
    }
  }

  virtual bool send_outgoing(const RNS::Bytes& data) override {
    try {
      if (ethernet_initialized && tcp_client.connected()) {
        TRACEF("TCPClientInterface.send_outgoing: (%u bytes)", data.size());
        tcp_client.write(HDLC_FLAG);
        const uint8_t* buf = data.data();
        size_t n = data.size();
        for (size_t i = 0; i < n; i++) {
          uint8_t c = buf[i];
          if (c == HDLC_FLAG || c == HDLC_ESC) {
            uint8_t esc[2] = { HDLC_ESC, (uint8_t)(c ^ HDLC_ESC_MASK) };
            tcp_client.write(esc, 2);
          } else {
            tcp_client.write(c);
          }
        }
        tcp_client.write(HDLC_FLAG);
      }
      InterfaceImpl::handle_outgoing(data);
    }
    catch (const std::bad_alloc&) {
      ERROR("TCPClientInterface::send_outgoing: bad_alloc - out of memory");
      return false;
    }
    catch (std::exception& e) {
      ERRORF("TCPClientInterface::send_outgoing: %s", e.what());
      return false;
    }
    return true;
  }

private:
  void _attempt_connect() {
    _last_reconnect = millis();
    if (tcp_client.connected()) tcp_client.stop();
    tcp_client.setConnectionTimeout(TCP_CONNECT_TIMEOUT_MS);
    INFOF("TCPClientInterface: connecting to %s:%u", _host, (unsigned)_port);
    if (tcp_client.connect(_host, _port)) {
      INFO("TCPClientInterface: connected");
      _connected = true;
      _rx_len    = 0;
      _in_frame  = false;
      _escape    = false;
    } else {
      _connected = false;
    }
  }

  void _feed_byte(uint8_t b) {
    if (b == HDLC_FLAG) {
      if (_in_frame && _rx_len > 0) {
        try {
          RNS::Bytes frame(_rx_buf, _rx_len);
          TRACEF("TCPClientInterface.handle_incoming: (%u bytes)", (unsigned)_rx_len);
          InterfaceImpl::handle_incoming(frame);
        }
        catch (const std::bad_alloc&) {
          ERROR("TCPClientInterface::handle_incoming: bad_alloc - out of memory");
        }
        catch (std::exception& e) {
          ERRORF("TCPClientInterface::handle_incoming: %s", e.what());
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
    if (_rx_len < TCP_RX_BUF_SIZE) {
      _rx_buf[_rx_len++] = b;
    } else {
      _in_frame = false;
      _rx_len   = 0;
      _escape   = false;
    }
  }

private:
  const char* _host;
  uint16_t    _port;
  bool        _connected       = false;
  uint32_t    _last_reconnect  = 0;
  uint8_t     _rx_buf[TCP_RX_BUF_SIZE];
  size_t      _rx_len          = 0;
  bool        _in_frame        = false;
  bool        _escape          = false;
};

#endif
