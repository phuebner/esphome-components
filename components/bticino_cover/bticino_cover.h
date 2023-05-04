#pragma once

#include "esphome/core/component.h"
#include "esphome/components/cover/cover.h"
#include "../bticino_bus/bticino_bus.h"

namespace esphome {
namespace bticino {

class BticinoCover : public cover::Cover, public Component, public bticino::BticinoDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_open_duration(uint32_t open_duration) { this->open_duration_ = open_duration; }
  void set_close_duration(uint32_t close_duration) { this->close_duration_ = close_duration; }
  void set_assumed_state(bool value) { this->assumed_state_ = value; }

  cover::CoverTraits get_traits() override;

 protected:
  void control(const cover::CoverCall &call) override;
  void on_bus_receive(const std::vector<uint8_t> &data) override;
  bool is_at_target_() const;
  void start_direction_(cover::CoverOperation dir);
  void recompute_position_();

  uint32_t open_duration_;
  uint32_t close_duration_;
  bool assumed_state_{false};

  uint32_t last_recompute_time_{0};
  uint32_t start_dir_time_{0};
  uint32_t last_publish_time_{0};
  float target_position_{0};

  bool externally_triggered_{false};

  cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};

 private:
  unsigned char _calculateChecksum(unsigned char *byteArray, int size);
  void _buildCommand(unsigned char *byteArray, int size, unsigned char *output);
  void sendSCSCommand(esphome::cover::CoverOperation op);
};

}  // namespace bticino
}  // namespace esphome
