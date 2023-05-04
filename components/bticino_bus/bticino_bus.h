
#pragma once

#include "esphome/core/component.h"

#include <vector>

namespace esphome {
namespace bticino {

class BticinoDevice;
class BticinoCommand;

class BticinoBus : public Component {
 public:
  BticinoBus() = default;

  void setup() override;
  void loop() override;
  void dump_config() override;

  void register_device(BticinoDevice *device) { this->devices_.push_back(device); }

  void send_raw(const std::vector<uint8_t> &payload);

 protected:
  std::vector<BticinoDevice *> devices_;
};

class BticinoDevice {
 public:
  void set_parent(BticinoBus *parent) { this->parent_ = parent; }
  void set_address(uint8_t address) { this->address_ = address; }

  virtual void on_bus_receive(const std::vector<uint8_t> &data) = 0;

  void send_raw(const std::vector<uint8_t> &payload) { this->parent_->send_raw(payload); }

 protected:
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
