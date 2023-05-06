
#pragma once

#include "esphome/core/component.h"

#include <vector>

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

  void register_device(BticinoDevice *device) { this->devices_.push_back(device); }

  bool parse_bticino_byte_(uint8_t byte);

  void send(uint8_t address, uint8_t destination, uint8_t function, uint8_t command);
  void send_raw(const std::vector<uint8_t> &payload);

 protected:
  uint8_t calculate_checksum_(const uint8_t *byteArray, uint8_t length);

  uint32_t last_rx_time_{0};
  uint32_t last_tx_time_{0};
  std::vector<uint8_t> last_tx_buffer_;
  uint8_t retry_counter_{0};
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
  void send_raw(const std::vector<uint8_t> &payload) { this->parent_->send_raw(payload); }

 protected:
  friend BticinoBus;

  BticinoBus *parent_;
  uint8_t address_;
};

class BticinoCommand {
 private:
  uint8_t Address;
  uint8_t Room;
  uint8_t Action;
  uint8_t Direction;

 public:
  BticinoCommand(uint8_t Address, uint8_t Room, uint8_t Action, uint8_t Direction);
};

}  // namespace bticino
}  // namespace esphome
