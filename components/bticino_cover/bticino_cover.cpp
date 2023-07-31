#include "esphome/core/log.h"
#include "bticino_cover.h"
#include <Arduino.h>
#include <esp32-hal-uart.h>
#include "soc/uart_reg.h"

namespace esphome {
namespace bticino {

static const char *TAG = "bticino.cover";

static const uint8_t COMMAND_UP = 0x09;
static const uint8_t COMMAND_DOWN = 0x08;
static const uint8_t COMMAND_STOP = 0x0a;

using namespace esphome::cover;

void BticinoCover::setup() {
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    this->position = 0.5f;
  }
}

void BticinoCover::loop() {
  const uint32_t now = millis();

  if (this->current_operation == COVER_OPERATION_IDLE)
    return;

  // Recompute position every loop cycle
  this->recompute_position_();

  if (this->is_at_target_()) {
    if ((this->current_operation == COVER_OPERATION_OPENING && this->target_position_ == COVER_OPEN) ||
        (this->current_operation == COVER_OPERATION_CLOSING && this->target_position_ == COVER_CLOSED)) {
      // Don't trigger stop, let the cover stop by itself.
      this->start_direction_(COVER_OPERATION_IDLE);
    } else {
      if (externally_triggered_ == false) {
        this->sendSCSCommand(COVER_OPERATION_IDLE);
      }
      this->start_direction_(COVER_OPERATION_IDLE);
    }
    this->publish_state();
  }
  // Send current position every second
  if (now - this->last_publish_time_ > 1000) {
    this->publish_state(false);
    this->last_publish_time_ = now;
  }
}

float BticinoCover::get_setup_priority() const { return setup_priority::DATA; }

void BticinoCover::dump_config() {
  LOG_COVER("", "Bticino Cover", this);
  ESP_LOGCONFIG(TAG, "  Target: %s", this->name_);
  ESP_LOGCONFIG(TAG, "  Address: 0x%02x", this->address_);
  ESP_LOGCONFIG(TAG, "  Open Duration: %.1fs", this->open_duration_ / 1e3f);
  ESP_LOGCONFIG(TAG, "  Close Duration: %.1fs", this->close_duration_ / 1e3f);
  ESP_LOGCONFIG(TAG, "  Tilt Duration: %.1fs", this->tilt_duration_ / 1e3f);
}

CoverTraits BticinoCover::get_traits() {
  auto traits = CoverTraits();
  traits.set_is_assumed_state(this->assumed_state_);
  traits.set_supports_position(true);
  traits.set_supports_tilt(true);
  traits.set_supports_stop(true);
  return traits;
}

void BticinoCover::control(const cover::CoverCall &call) {
  unsigned char command = 0x00;
  if (call.get_stop()) {
    this->externally_triggered_ = false;
    this->start_direction_(COVER_OPERATION_IDLE);
    this->sendSCSCommand(COVER_OPERATION_IDLE);
    this->publish_state();
  }

  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    ESP_LOGD(TAG, "position call - target: %f", pos);
    if (pos == this->position) {
      // already at target
      // for covers with built in end stop, we should send the command again
      if (pos == COVER_OPEN || pos == COVER_CLOSED) {
        auto op = pos == COVER_CLOSED ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING;
        this->target_position_ = pos;
        this->target_tilt_ = op == COVER_OPERATION_CLOSING ? 1.0 : 0.0;
        this->externally_triggered_ = false;
        this->start_direction_(op);
        this->sendSCSCommand(op);
      }
    } else {
      auto op = pos < this->position ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING;
      this->target_position_ = pos;
      this->target_tilt_ = op == COVER_OPERATION_CLOSING ? 1.0 : 0.0;
      this->externally_triggered_ = false;
      this->start_direction_(op);
      this->sendSCSCommand(op);
    }
  }

  if (call.get_tilt().has_value()) {
    auto tilt = *call.get_tilt();
    ESP_LOGD(TAG, "tilt call - target: %f", tilt);
    if (tilt != this->tilt) {
      auto op = tilt < this->tilt ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING;
      this->target_position_ = this->position;
      this->target_tilt_ = tilt;
      this->externally_triggered_ = false;
      this->sendSCSCommand(op);
      this->start_direction_(op);
    }
  }
}

void BticinoCover::on_bus_receive(uint8_t function, uint8_t command) {
  const uint32_t now = millis();
  ESP_LOGD(TAG, "'%s' handling received command", this->name_.c_str());

  switch (command) {
    case COMMAND_UP:
      this->target_position_ = COVER_OPEN;
      this->externally_triggered_ = true;
      start_direction_(COVER_OPERATION_OPENING);
      break;
    case COMMAND_DOWN:
      this->target_position_ = COVER_CLOSED;
      this->externally_triggered_ = true;
      start_direction_(COVER_OPERATION_CLOSING);
      break;
    case COMMAND_STOP:
      this->externally_triggered_ = true;
      start_direction_(COVER_OPERATION_IDLE);
      break;
    default:
      break;
  }
  this->publish_state();
  this->last_publish_time_ = now;
}

void BticinoCover::sendSCSCommand(esphome::cover::CoverOperation op) {
  unsigned char command = 0x00;

  switch (op) {
    case COVER_OPERATION_OPENING:
      command = COMMAND_UP;
      break;
    case COVER_OPERATION_CLOSING:
      command = COMMAND_DOWN;
      break;
    case COVER_OPERATION_IDLE:
      command = COMMAND_STOP;
      break;
    default:
      break;
  }
  // Check if we have a valid command
  if (command == 0x00)
    return;  // do nothing

  this->send(this->address_, 0x00, FUNCTION_COVER, command);
}

bool BticinoCover::is_at_target_() const {
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      return this->position >= this->target_position_ && this->tilt >= this->target_tilt_;
    case COVER_OPERATION_CLOSING:
      return this->position <= this->target_position_ && this->tilt <= this->target_tilt_;
    case COVER_OPERATION_IDLE:
    default:
      return true;
  }
}

void BticinoCover::start_direction_(CoverOperation dir) {
  if (dir == this->current_operation && dir != COVER_OPERATION_IDLE)
    return;

  this->recompute_position_();
  switch (dir) {
    case COVER_OPERATION_IDLE:
      break;
    case COVER_OPERATION_OPENING:
      this->last_operation_ = dir;
      break;
    case COVER_OPERATION_CLOSING:
      this->last_operation_ = dir;
      break;
    default:
      return;
  }

  this->current_operation = dir;

  const uint32_t now = millis();
  this->start_dir_time_ = now;
  this->last_recompute_time_ = now;
}

void BticinoCover::recompute_position_() {
  if (this->current_operation == COVER_OPERATION_IDLE)
    return;

  float dir;
  float action_dur;
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      dir = 1.0f;
      action_dur = this->open_duration_;
      break;
    case COVER_OPERATION_CLOSING:
      dir = -1.0f;
      action_dur = this->close_duration_;
      break;
    default:
      return;
  }

  const uint32_t now = millis();
  this->position += dir * (now - this->last_recompute_time_) / action_dur;
  this->position = clamp(this->position, 0.0f, 1.0f);

  this->tilt += dir * (float) (now - this->last_recompute_time_) / (float) this->tilt_duration_;
  this->tilt = clamp(this->tilt, 0.0f, 1.0f);

  this->last_recompute_time_ = now;
}

}  // namespace bticino
}  // namespace esphome
