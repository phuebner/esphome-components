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

void BticinoBus::loop() {}

void BticinoBus::dump_config() {}

void BticinoBus::send_raw(const std::vector<uint8_t> &payload) {
  WRITE_PERI_REG(UART_CONF0_REG(1), READ_PERI_REG(UART_CONF0_REG(1)) | UART_IRDA_TX_EN);  // enable UART_IRDA_TX_EN

  // Build string
  char hex_str[payload.size() * 3 + 1] = {0};
  for (int i = 0; i < payload.size(); i++) {
    sprintf(&hex_str[i * 3], "%02x ", payload.at(i));
  }

  ESP_LOGD(TAG, "Sending command: %s", hex_str);
  Serial1.write(payload.data(), payload.size());
  Serial1.flush(true);
  WRITE_PERI_REG(UART_CONF0_REG(1), READ_PERI_REG(UART_CONF0_REG(1)) & ~UART_IRDA_TX_EN);  // disable
                                                                                           // UART_IRDA_TX_EN
  delay(80);  // Delay needed to not have conflicts with the relay messages on the SCS bus
}

}  // namespace bticino
}  // namespace esphome
