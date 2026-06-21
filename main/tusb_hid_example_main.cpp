/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <Arduino.h>

extern "C" {
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include <stdlib.h>
}

extern "C" {
void app_main(void);
}

#define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default
static const char *TAG = "example";

/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN                                                    \
  (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))};

/**
 * @brief String descriptor
 */
const char *hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},    // 0: is supported language is English (0x0409)
    "TinyUSB",               // 1: Manufacturer
    "TinyUSB Device",        // 2: Product
    "123456",                // 3: Serials, should use chip ID
    "Example HID interface", // 4: HID
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1
 * HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length,
    // attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP
    // In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16,
                       10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
  return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;
  return 0;
}

// Invoked when received SET_REPORT control request
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {}

/********* Serial2 → Serial forwarding ***************/

/**
 * @brief FreeRTOS task: baca data dari Serial2 (HID reader, RX=17 TX=18),
 *        forward ke Serial default (USB/UART0) sebagai passthrough.
 */
static void serial2_forward_task(void *arg) {
  while (true) {
    if (Serial2.available()) {
      // Baca semua byte yang tersedia dan kirim ke Serial default
      while (Serial2.available()) {
        char c = (char)Serial2.read();
        Serial.write(c);
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); // yield minimal
  }
}

/********* Application ***************/

typedef enum {
  MOUSE_DIR_RIGHT,
  MOUSE_DIR_DOWN,
  MOUSE_DIR_LEFT,
  MOUSE_DIR_UP,
  MOUSE_DIR_MAX,
} mouse_dir_t;

#define DISTANCE_MAX 125
#define DELTA_SCALAR 5

static void app_send_hid_demo(void) {
  ESP_LOGI(TAG, "Sending Keyboard report");
  uint8_t keycode[6] = {HID_KEY_A};
  tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
  vTaskDelay(pdMS_TO_TICKS(50));
  tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);

  ESP_LOGI(TAG, "Sending Mouse report");
  int16_t delta_x;
  int16_t delta_y;
}

void app_main(void) {
  // Serial default (UART0) — untuk monitor / forward output
  Serial.begin(115200);
  while (!Serial) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  // Serial2 — menerima data dari ESP32 HID reader (TX=18 di sisi ini = RX
  // reader) RX = pin 17, TX = pin 18, baud 115200
  Serial2.begin(115200, SERIAL_8N1, 17, 18);

  Serial.println("[tusb_hid] Serial2 initialized: RX=17, TX=18 @ 115200");

  // Task untuk forward Serial2 → Serial default
  xTaskCreate(serial2_forward_task, "serial2_fwd", 2048, NULL, 5, NULL);

  ESP_LOGI(TAG, "USB initialization");
  tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();

  tusb_cfg.descriptor.device = NULL;
  tusb_cfg.descriptor.full_speed_config = hid_configuration_descriptor;
  tusb_cfg.descriptor.string = hid_string_descriptor;
  tusb_cfg.descriptor.string_count =
      sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]);
#if (TUD_OPT_HIGH_SPEED)
  tusb_cfg.descriptor.high_speed_config = hid_configuration_descriptor;
#endif // TUD_OPT_HIGH_SPEED

  ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
  ESP_LOGI(TAG, "USB initialization DONE");

  while (1) {
    if (tud_mounted()) {
      static bool send_hid_data = true;
      if (send_hid_data) {
        app_send_hid_demo();
      }
      send_hid_data = !gpio_get_level(APP_BUTTON);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
