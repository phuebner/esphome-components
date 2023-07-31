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
  if (now - this->last_rx_time_ > 60) {
    if (this->rx_buffer_.size() > 0) {
      ESP_LOGW(TAG, "Dropping incomplete rx buffer");
      this->rx_buffer_.clear();
    }
  }

  // Wait at least 20 ms after we received something to make sure there is not anoter repeated message comming
  if (now - this->last_rx_time_ > 30 && !Serial1.available() && this->rx_buffer_.size() == 0) {
    send_next_();
  }

  uint8_t timeout_counter = 8;  // timout so that our loop is not locking up
  while (Serial1.available() && timeout_counter > 0) {
    uint8_t byte;
    byte = Serial1.read();
    this->last_rx_time_ = now;
    if (this->parse_bticino_byte_(byte)) {
      this->last_rx_time_ = now;
    } else {
      this->rx_buffer_.clear();
    }
    timeout_counter--;
  }
}

float BticinoBus::get_setup_priority() const { return setup_priority::BUS; }

void BticinoBus::dump_config() {}

bool BticinoBus::parse_bticino_byte_(uint8_t byte) {
  const uint32_t now = millis();

  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  const uint8_t *raw = &this->rx_buffer_[0];

  if (at == 0) {
    switch (raw[0]) {
      case START_BYTE:
        ESP_LOGV(TAG, "=== Start byte ===");
        ESP_LOGV(TAG, "Bticino received byte  %d (0x%02x)", byte, byte);
        return true;  // start byte valid -> continue capturing bytes
        break;
      case HEARTBEAT_BYTE:
        ESP_LOGVV(TAG, "Bus heartbeat received");
        return false;
        break;
      case ACK_BYTE:
        // Received an acknowledge
        ESP_LOGD(TAG, "Bticino received acknowledge");
        this->retry_counter_ = 0;  // no need to retry sending a package
        return false;
        break;
      default:
        return false;  // reset buffer
    }
  }

  ESP_LOGV(TAG, "Bticino received byte  %d (0x%02x)", byte, byte);

  // continue capturing until we have enough bytes to evaluate the package
  if (at < 6)
    return true;

  uint8_t address = raw[1];
  uint8_t destination = raw[2];
  uint8_t function_code = raw[3];
  uint8_t command_code = raw[4];
  uint8_t received_checksum = raw[5];

  uint8_t computed_checksum = calculate_checksum_(&raw[1], 4);

  // Build logging string
  char hex_str[this->rx_buffer_.size() * 3 + 1] = {0};
  for (int i = 0; i < this->rx_buffer_.size(); i++) {
    sprintf(&hex_str[i * 3], "%02x ", this->rx_buffer_.at(i));
  }

  ESP_LOGD(TAG, "Command received: %s", hex_str);

  // Ignore "copies" of commands that we just sent.
  if (now - this->last_tx_time_ < 300 && this->rx_buffer_ == this->last_tx_buffer_) {
    ESP_LOGV(TAG, "Copy of sent command ignored");
    return false;
  }
  // Ignore relay commands of a command that we just sent
  if (now - this->last_tx_time_ < 600 && this->rx_buffer_ == this->relay_command_(this->last_tx_buffer_)) {
    ESP_LOGV(TAG, "Relay of sent command ignored");
    return false;
  }

  // Ignore all relay commands for a set period
  if (now - this->last_tx_time_ < 600 && address == ADDRESS_RELAY_CALL) {
    ESP_LOGW(TAG, "Strange relay command ignored");
    return false;
  }

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

  // Notify devices of the received package
  for (auto *device : this->devices_) {
    if (address == device->address_ || address == ADDRESS_RELAY_CALL && destination == device->address_ ||
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

std::vector<uint8_t> BticinoBus::relay_command_(const std::vector<uint8_t> &package) {
  uint8_t address = package[1];
  uint8_t function = package[3];
  uint8_t command = package[4];

  uint8_t checksum = ADDRESS_RELAY_CALL ^ address ^ function ^ command;
  std::vector<uint8_t> relay_package{START_BYTE, ADDRESS_RELAY_CALL, address, function, command, checksum, STOP_BYTE};
  return relay_package;
}

void BticinoBus::send(uint8_t address, uint8_t destination, uint8_t function, uint8_t command) {
  uint8_t checksum = address ^ destination ^ function ^ command;
  std::vector<uint8_t> package{START_BYTE, address, destination, function, command, checksum, STOP_BYTE};
  this->queue_send_(package);
}

void BticinoBus::queue_send_(const std::vector<uint8_t> &package) {
  // Build logging string
  char hex_str[package.size() * 3 + 1] = {0};
  for (int i = 0; i < package.size(); i++) {
    sprintf(&hex_str[i * 3], "%02x ", package.at(i));
  }
  ESP_LOGD(TAG, "Add command %s to queue", hex_str);

  send_queue_.push_back(package);
  ESP_LOGD(TAG, "%i commands queued", send_queue_.size());
}

void BticinoBus::send_next_() {
  int32_t last_send = millis() - this->last_tx_time_;
  if ((last_send > this->send_throttle_) && !send_queue_.empty()) {
    auto &package = send_queue_.front();

    if (this->retry_counter_ > 0) {
      this->send_raw_(package);
    } else {
      // Command either successfully sent or reached maximum retries
      ESP_LOGD(TAG, "Remove send command from queue");
      this->retry_counter_ = this->max_retries_;
      send_queue_.pop_front();
    }
  }
}

void BticinoBus::send_raw_(const std::vector<uint8_t> &package) {
  const uint32_t now = millis();
  this->last_tx_time_ = now;
  this->last_tx_buffer_ = package;

  WRITE_PERI_REG(UART_CONF0_REG(1), READ_PERI_REG(UART_CONF0_REG(1)) | UART_IRDA_TX_EN);  // enable UART_IRDA_TX_EN

  // Build logging string
  char hex_str[package.size() * 3 + 1] = {0};
  for (int i = 0; i < package.size(); i++) {
    sprintf(&hex_str[i * 3], "%02x ", package.at(i));
  }
  if (this->retry_counter_ < this->max_retries_) {
    ESP_LOGD(TAG, "Retry sending command: %s", hex_str);
  } else {
    ESP_LOGD(TAG, "Sending command: %s", hex_str);
  }
  Serial1.write(package.data(), package.size());
  Serial1.flush(true);
  WRITE_PERI_REG(UART_CONF0_REG(1), READ_PERI_REG(UART_CONF0_REG(1)) & ~UART_IRDA_TX_EN);  // disable

  if (this->retry_counter_)
    this->retry_counter_--;
}

}  // namespace bticino
}  // namespace esphome
