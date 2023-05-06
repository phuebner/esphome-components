#include "esphome/core/log.h"
#include <Arduino.h>
#include <esp32-hal-uart.h>
#include "soc/uart_reg.h"

#include "bticino_bus.h"

namespace esphome {
namespace bticino {

static const char *TAG = "bticino.bus";

void BticinoBus::setup() {
  Serial1.begin(9600, SERIAL_8N1, 16, 5);
  pinMode(16, INPUT_PULLDOWN);  // disable the pullup that is enabled by the driver
  WRITE_PERI_REG(UART_CONF0_REG(1), READ_PERI_REG(UART_CONF0_REG(1)) | UART_IRDA_EN);    // Enable IRDA mode}
  WRITE_PERI_REG(UART_CONF0_REG(1), READ_PERI_REG(UART_CONF0_REG(1)) | UART_IRDA_DPLX);  // Enable IRDA Duplex
}

void BticinoBus::loop() {
  const uint32_t now = millis();

  // Clear the rx_buffer if there is a long pause between subsequent bytes
  if (now - this->last_rx_time_ > 30) {
    this->rx_buffer_.clear();
    this->last_rx_time_ = now;
  }

  // Resend messages until retry_counter_ is 0
  // if (this->retry_counter_ > 0 && this->last_tx_time_ > 11) {
  //   ESP_LOGD(TAG, "Retry sending command");
  //   this->send_raw(this->last_tx_buffer_);
  // }

  while (Serial1.available()) {
    uint8_t byte;
    byte = Serial1.read();
    if (this->parse_bticino_byte_(byte)) {
      this->last_rx_time_ = now;
    } else {
      this->rx_buffer_.clear();
    }
  }
}

void BticinoBus::dump_config() {}

bool BticinoBus::parse_bticino_byte_(uint8_t byte) {
  const uint32_t now = millis();
  // Ignore "copies" of commands that we just sent.
  if (now - this->last_tx_time_ < 300)
    return false;

  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  const uint8_t *raw = &this->rx_buffer_[0];

  if (at == 0 && raw[0] == HEARTBEAT_BYTE) {
    // Heartbeat received
    ESP_LOGVV(TAG, "Bus heartbeat received");
    return false;
  }

  if (at == 0 && raw[0] == ACK_BYTE) {
    // Received an acknowledge
    ESP_LOGV(TAG, "Bticino received acknowledge");
    this->retry_counter_ = 0;  // no need to retry sending a package
    return false;
  }

  ESP_LOGV(TAG, "Bticino received byte  %d (0x%02x)", byte, byte);
  if (at == 0 && raw[0] == START_BYTE)
    return true;  // start byte valid -> continue capturing bytes

  // continue capturing until we have enough bytes to evaluate the package
  if (at < 6)
    return true;

  uint8_t address = raw[1];
  uint8_t destination = raw[2];
  uint8_t function_code = raw[3];
  uint8_t command_code = raw[4];
  uint8_t received_checksum = raw[5];

  uint8_t computed_checksum = calculate_checksum_(raw, at);

  if (computed_checksum != received_checksum) {
    ESP_LOGW(TAG, "RX package checksum check failed! 0x%02x != 0x%02x", computed_checksum, received_checksum);
    return false;
  }

  if (raw[6] != STOP_BYTE) {
    ESP_LOGW(TAG, "RX package wrong stop byte: 0x%02x ", raw[6]);
    return false;
  }

  if (function_code != FUNCTION_COVER) {
    ESP_LOGW(TAG, "RX package unknown function code: 0x%02x ", function_code);
    return false;
  }

  // Build logging string
  char hex_str[this->rx_buffer_.size() * 3 + 1] = {0};
  for (int i = 0; i < this->rx_buffer_.size(); i++) {
    sprintf(&hex_str[i * 3], "%02x ", this->rx_buffer_.at(i));
  }

  ESP_LOGD(TAG, "Command received: %s", hex_str);

  // Notify devices of the received package
  for (auto *device : this->devices_) {
    if (address == device->address_ ||
        address == ADDRESS_ROOM_CALL && destination == ((device->address_ >> 4) & 0x0F) ||
        address == ADDRESS_GENERAL_CALL) {
      device->on_bus_receive(function_code, command_code);
    }
  }
  // return false to reset buffer
  return false;
}

uint8_t BticinoBus::calculate_checksum_(const uint8_t *byteArray, uint8_t length) {
  uint8_t checksum = 0;
  for (int i = 0; i < length; ++i) {
    checksum ^= byteArray[i];
  }
  return checksum;
}

void BticinoBus::send(uint8_t address, uint8_t destination, uint8_t function, uint8_t command) {
  uint8_t checksum = address ^ destination ^ function ^ command;
  std::vector<uint8_t> payload{START_BYTE, address, destination, function, command, checksum, STOP_BYTE};
  this->retry_counter_ = 3;
  this->last_tx_buffer_ = payload;
  this->send_raw(payload);
}

void BticinoBus::send_raw(const std::vector<uint8_t> &payload) {
  const uint32_t now = millis();
  this->last_tx_time_ = now;

  WRITE_PERI_REG(UART_CONF0_REG(1), READ_PERI_REG(UART_CONF0_REG(1)) | UART_IRDA_TX_EN);  // enable UART_IRDA_TX_EN

  // Build logging string
  char hex_str[payload.size() * 3 + 1] = {0};
  for (int i = 0; i < payload.size(); i++) {
    sprintf(&hex_str[i * 3], "%02x ", payload.at(i));
  }

  ESP_LOGD(TAG, "Sending command: %s", hex_str);
  Serial1.write(payload.data(), payload.size());
  Serial1.flush(true);
  WRITE_PERI_REG(UART_CONF0_REG(1), READ_PERI_REG(UART_CONF0_REG(1)) & ~UART_IRDA_TX_EN);  // disable

  if (this->retry_counter_)
    this->retry_counter_--;
}

}  // namespace bticino
}  // namespace esphome
