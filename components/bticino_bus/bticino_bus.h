
#pragma once

#include "esphome/core/component.h"

#include <vector>
#include <list>

namespace esphome {
namespace bticino {

static const uint8_t START_BYTE = 0xA8;
static const uint8_t STOP_BYTE = 0xA3;
static const uint8_t ACK_BYTE = 0xA5;
static const uint8_t HEARTBEAT_BYTE = 0xFF;

static const uint8_t ADDRESS_GENERAL_CALL = 0xB1;
static const uint8_t ADDRESS_ROOM_CALL = 0xB3;
static const uint8_t ADDRESS_RELAY_CALL = 0xB8;

static const uint8_t FUNCTION_COVER = 0x12;

class BticinoDevice;
class BticinoCommand;

class BticinoBus : public Component {
 public:
  BticinoBus() = default;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void register_device(BticinoDevice *device) { this->devices_.push_back(device); }

  bool parse_bticino_byte_(uint8_t byte);

  void send(uint8_t address, uint8_t destination, uint8_t function, uint8_t command);

 protected:
  void send_next_();
  void queue_send_(const std::vector<uint8_t> &payload);
  void send_raw_(const std::vector<uint8_t> &payload);
  uint8_t calculate_checksum_(const uint8_t *byteArray, uint8_t length);
  std::vector<uint8_t> relay_command_(const std::vector<uint8_t> &payload);

  /// Holds the time in ms that should be waited for between individual sends
  uint8_t send_throttle_{120};
  uint32_t last_rx_time_{0};
  uint32_t last_tx_time_{0};
  std::vector<uint8_t> last_tx_buffer_;
  uint8_t retry_counter_{4};
  uint8_t max_retries_{4};

  /// @brief Hold the pending request to be sent
  std::list<std::vector<uint8_t>> send_queue_;

  std::vector<uint8_t> rx_buffer_;
  std::vector<BticinoDevice *> devices_;
};

class BticinoDevice {
 public:
  void set_parent(BticinoBus *parent) { this->parent_ = parent; }
  void set_address(uint8_t address) { this->address_ = address; }

  virtual void on_bus_receive(uint8_t function, uint8_t command) = 0;

  void send(uint8_t address, uint8_t destination, uint8_t function, uint8_t command) {
    this->parent_->send(address, destination, function, command);
  }

 protected:
  friend BticinoBus;

  BticinoBus *parent_;
  uint8_t address_;
};

}  // namespace bticino
}  // namespace esphome
