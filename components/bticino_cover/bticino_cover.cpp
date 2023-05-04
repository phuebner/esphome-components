#include "esphome/core/log.h"
#include "bticino_cover.h"
#include <Arduino.h>
#include <esp32-hal-uart.h>
#include "soc/uart_reg.h"

namespace esphome {
namespace bticino {

static const char *TAG = "bticino.cover";

static const char START_BYTE = 0xA8;
static const char STOP_BYTE = 0xA3;

static const char ADDRESS_GENERAL_CALL = 0xB1;
static const char ADDRESS_ROOM_CALL = 0xB3;
static const char ADDRESS_RELAY_CALL = 0xB8;

static const char CMD_BLIND = 0x12;

static const char CMD2_UP = 0x09;
static const char CMD2_DOWN = 0x08;
static const char CMD2_STOP = 0x0a;

using namespace esphome::cover;

void BticinoCover::setup() {
  // Serial1.begin(9600, SERIAL_8N1, 16, 5);
  // pinMode(16, INPUT_PULLDOWN);  // disable the pullup that is enabled by the driver
  // WRITE_PERI_REG(UART_CONF0_REG(1), READ_PERI_REG(UART_CONF0_REG(1)) | UART_IRDA_EN);    // Enable IRDA mode}
  // WRITE_PERI_REG(UART_CONF0_REG(1), READ_PERI_REG(UART_CONF0_REG(1)) | UART_IRDA_DPLX);  // Enable IRDA Duplex
  // // mode} WRITE_PERI_REG(UART_CONF0_REG(1), READ_PERI_REG(UART_CONF0_REG(1)) | UART_IRDA_RX_INV);  // Enable IRDA
  // // Duplex mode}

  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    this->position = 0.5f;
  }
}

void BticinoCover::loop() {
  const uint32_t now = millis();

  // // Monitor bus to catch commands sent by other sources (e.g. push buttons)
  // if (Serial1.available()) {
  //   char c = Serial1.read();
  //   static char buffer[10];
  //   static int i = 0;
  //   if (c == START_BYTE) {
  //     i = 0;
  //     buffer[i++] = c;
  //   } else if (i > 0 && i < 9) {
  //     buffer[i++] = c;
  //     if (c == STOP_BYTE) {
  //       buffer[i] = '\0';
  //       if (i >= 4) {
  //         if ((buffer[1] == this->address_ &&
  //              (now - this->start_dir_time_) > 500) || /* Normal addrss call and not self-triggered*/
  //             (buffer[1] == ADDRESS_ROOM_CALL && buffer[2] == ((this->address_ >> 4) & 0x0F)) || /* Room call*/
  //             buffer[1] == ADDRESS_GENERAL_CALL) {                                               /* General call */
  //           ESP_LOGD("bticino", "Received byte sequence: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x, 0x%02x",
  //           buffer[0],
  //                    buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6]);

  //           switch (buffer[4]) {
  //             case CMD2_UP:
  //               this->target_position_ = COVER_OPEN;
  //               this->externally_triggered_ = true;
  //               start_direction_(COVER_OPERATION_OPENING);
  //               break;
  //             case CMD2_DOWN:
  //               this->target_position_ = COVER_CLOSED;
  //               this->externally_triggered_ = true;
  //               start_direction_(COVER_OPERATION_CLOSING);
  //               break;
  //             case CMD2_STOP:
  //               this->externally_triggered_ = true;
  //               start_direction_(COVER_OPERATION_IDLE);
  //               break;
  //             default:
  //               break;
  //           }
  //         }
  //         i = 0;
  //       }
  //     }
  //   }
  // }

  if (this->current_operation == COVER_OPERATION_IDLE)
    return;

  // Recompute position every loop cycle
  this->recompute_position_();

  if (this->is_at_target_()) {
    if (externally_triggered_ == false) {
      this->sendSCSCommand(COVER_OPERATION_IDLE);
    }
    this->start_direction_(COVER_OPERATION_IDLE);
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
}

CoverTraits BticinoCover::get_traits() {
  auto traits = CoverTraits();
  traits.set_is_assumed_state(this->assumed_state_);
  traits.set_supports_position(true);
  traits.set_supports_tilt(false);

  return traits;
}

unsigned char BticinoCover::_calculateChecksum(unsigned char *byteArray, int size) {
  unsigned char checksum = 0;
  for (int i = 0; i < size; ++i) {
    checksum ^= byteArray[i];
  }
  return checksum;
}

void BticinoCover::_buildCommand(unsigned char *byteArray, int size, unsigned char *output) {
  unsigned char checksum = _calculateChecksum(byteArray, size);

  unsigned char newByteArray[size + 3];
  output[0] = START_BYTE;
  for (int i = 0; i < size; ++i) {
    output[i + 1] = byteArray[i];
  }
  output[size + 1] = checksum;
  output[size + 2] = STOP_BYTE;
}

void BticinoCover::control(const cover::CoverCall &call) {
  unsigned char cmd2 = 0x00;
  if (call.get_stop()) {
    this->externally_triggered_ = false;
    this->start_direction_(COVER_OPERATION_IDLE);
    this->sendSCSCommand(COVER_OPERATION_IDLE);
  }

  if (call.get_position().has_value()) {
    auto pos = *call.get_position();

    ESP_LOGD("bticino", "==== Position Call - Target: %f", pos);
    auto op = pos <= this->position ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING;
    this->externally_triggered_ = false;
    this->target_position_ = pos;
    this->start_direction_(op);
    this->sendSCSCommand(op);
  }
}

void BticinoCover::on_bus_receive(const std::vector<uint8_t> &data) {}

void BticinoCover::sendSCSCommand(esphome::cover::CoverOperation op) {
  unsigned char cmd2 = 0x00;

  switch (op) {
    case COVER_OPERATION_OPENING:
      cmd2 = CMD2_UP;
      break;
    case COVER_OPERATION_CLOSING:
      cmd2 = CMD2_DOWN;
      break;
    case COVER_OPERATION_IDLE:
      cmd2 = CMD2_STOP;
      break;
    default:
      break;
  }

  unsigned char command[] = {this->address_, 0x00, CMD_BLIND, cmd2};

  unsigned char fullCommand[sizeof(command) + 3];
  _buildCommand(command, sizeof(command), fullCommand);

  std::vector<uint8_t> payload((uint8_t *) fullCommand, (uint8_t *) fullCommand + sizeof(fullCommand));
  this->send_raw(payload);
}

bool BticinoCover::is_at_target_() const {
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      return this->position >= this->target_position_;
    case COVER_OPERATION_CLOSING:
      return this->position <= this->target_position_;
    case COVER_OPERATION_IDLE:
    default:
      return true;
  }
}

void BticinoCover::start_direction_(CoverOperation dir) {
  if (dir == this->current_operation && dir != COVER_OPERATION_IDLE)
    return;

  this->recompute_position_();
  // Trigger<> *trig;
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

  this->last_recompute_time_ = now;
}

}  // namespace bticino
}  // namespace esphome
