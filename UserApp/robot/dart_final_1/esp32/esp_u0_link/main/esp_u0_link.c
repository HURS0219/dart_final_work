/*
 * esp_u0_link.c — ESP32-S3 <-> STM32H743 UART0 链路测试 (ESP-IDF)
 * =============================================================================
 * UART0 @115200 8N1, TX=GPIO43 / RX=GPIO44 (ESP32-S3-DevKitC-1 "TX/RX" 排针)。
 * 控制台请设到 USB Serial/JTAG (sdkconfig.defaults 已设), 以释放 UART0。
 *
 * 正常模式: 收 "PING" -> 回 "PONG\n"; 每 500ms 主动发 "H\n"。
 * 自测模式(把 ESP_LINK_LOOPBACK 改为 1 重新编译): 用跳线短接 GPIO43<->GPIO44,
 *           周期发 "PING" 并检查回环, 打印 LOOPBACK PASS/FAIL。
 * =============================================================================
 */
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_PORT UART_NUM_0
#define PIN_TXD 43
#define PIN_RXD 44
#define BUF_SIZE 256

#define ESP_LINK_LOOPBACK 0

static const char *TAG = "esp_u0";

static void uart_init(void) {
  const uart_config_t cfg = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT, PIN_TXD, PIN_RXD, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));
}

void app_main(void) {
  uart_init();
  ESP_LOGI(TAG, "UART0 ready 115200 8N1  TX=GPIO%d RX=GPIO%d", PIN_TXD, PIN_RXD);

#if ESP_LINK_LOOPBACK
  uint32_t ok = 0, fail = 0;
  uint8_t rx[BUF_SIZE];
  while (1) {
    uart_write_bytes(UART_PORT, "PING\n", 5);
    int n = uart_read_bytes(UART_PORT, rx, sizeof(rx), pdMS_TO_TICKS(200));
    if (n > 0) {
      ok++;
      ESP_LOGI(TAG, "LOOPBACK PASS (%u), rx=%d", (unsigned)ok, n);
    } else {
      fail++;
      ESP_LOGW(TAG, "LOOPBACK FAIL (%u) - short GPIO%d<->GPIO%d ?", (unsigned)fail,
               PIN_TXD, PIN_RXD);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
#else
  uint8_t data[BUF_SIZE];
  char line[64];
  int li = 0;
  TickType_t last_hb = xTaskGetTickCount();
  while (1) {
    int n = uart_read_bytes(UART_PORT, data, sizeof(data), pdMS_TO_TICKS(20));
    for (int i = 0; i < n; i++) {
      char c = (char)data[i];
      if (c == '\n' || c == '\r') {
        if (li > 0) {
          line[li] = '\0';
          if (strcmp(line, "PING") == 0) {
            uart_write_bytes(UART_PORT, "PONG\n", 5);
            ESP_LOGI(TAG, "PING -> PONG");
          } else if (line[0] == 'F' && line[1] == ',') {
            ESP_LOGI(TAG, "telemetry: %s", line);
          } else {
            ESP_LOGI(TAG, "rx line: %s", line);
          }
          li = 0;
        }
      } else if (li < (int)sizeof(line) - 1) {
        line[li++] = c;
      }
    }
    if (xTaskGetTickCount() - last_hb >= pdMS_TO_TICKS(500)) {
      last_hb = xTaskGetTickCount();
      uart_write_bytes(UART_PORT, "H\n", 2);
    }
  }
#endif
}
