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

// Device descriptor
static const tusb_desc_device_t custom_device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, // USB 2.0 (same as default TinyUSB to avoid compatibility
                      // issues)
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 =
        64, // 64 bytes (same as default TinyUSB to avoid compatibility issues)
    .idVendor = 0x258A,  // Sino Wealth Electronic Ltd.
    .idProduct = 0x0051, // Gaming Mouse PID
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x00, // No serial
    .bNumConfigurations = 0x01};

// Mouse Report Descriptor (71 bytes)
const uint8_t hid_mouse_report_desc[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop Controls)
    0x09, 0x02,       // Usage (Mouse)
    0xA1, 0x01,       // Collection (Application)
    0x09, 0x01,       //   Usage (Pointer)
    0xA1, 0x00,       //   Collection (Physical)
    0x05, 0x09,       //     Usage Page (Buttons)
    0x19, 0x01,       //     Usage Minimum (1)
    0x29, 0x05,       //     Usage Maximum (5)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x01,       //     Logical Maximum (1)
    0x75, 0x01,       //     Report Size (1)
    0x95, 0x05,       //     Report Count (5)
    0x81, 0x02,       //     Input (Var)
    0x95, 0x03,       //     Report Count (3)
    0x81, 0x01,       //     Input (Const)
    0x05, 0x01,       //     Usage Page (Generic Desktop Controls)
    0x09, 0x30,       //     Usage (Direction-X)
    0x09, 0x31,       //     Usage (Direction-Y)
    0x16, 0x00, 0x80, //     Logical Minimum (-32768)
    0x26, 0xFF, 0x7F, //     Logical Maximum (+32767)
    0x75, 0x10,       //     Report Size (16)
    0x95, 0x02,       //     Report Count (2)
    0x81, 0x06,       //     Input (Var, Rel)
    0x09, 0x38,       //     Usage (Wheel)
    0x15, 0x80,       //     Logical Minimum (-128)
    0x25, 0x7F,       //     Logical Maximum (+127)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x01,       //     Report Count (1)
    0x81, 0x06,       //     Input (Var, Rel)
    0x05, 0x0C,       //     Usage Page (Consumer)
    0x0A, 0x38, 0x02, //     Usage (AC Pan)
    0x95, 0x01,       //     Report Count (1)
    0x81, 0x06,       //     Input (Var, Rel)
    0xC0,             //   End Collection
    0xC0              // End Collection
};

// Keyboard Report Descriptor (213 bytes)
const uint8_t hid_keyboard_report_desc[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop Controls)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (0x01)
    0x05, 0x07,       //   Usage Page (Keyboard)
    0x19, 0xE0,       //   Usage Minimum (224)
    0x29, 0xE7,       //   Usage Maximum (231)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x02,       //   Input (Var)
    0x95, 0x06,       //   Report Count (6)
    0x75, 0x08,       //   Report Size (8)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x05, 0x07,       //   Usage Page (Keyboard)
    0x19, 0x00,       //   Usage Minimum (0)
    0x2A, 0xFF, 0x00, //   Usage Maximum (255)
    0x81, 0x00,       //   Input ()
    0xC0,             // End Collection
    0x06, 0x0C, 0x00, // Usage Page (Consumer)
    0x09, 0x01,       // Usage (Consumer Control)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x02,       //   Report ID (0x02)
    0x25, 0x01,       //   Logical Maximum (1)
    0x15, 0x00,       //   Logical Minimum (0)
    0x75, 0x01,       //   Report Size (1)
    0x0A, 0xB5, 0x00, //   Usage (Scan Next Track)
    0x0A, 0xB6, 0x00, //   Usage (Scan Previous Track)
    0x0A, 0xB7, 0x00, //   Usage (Stop)
    0x0A, 0xCD, 0x00, //   Usage (Play/Pause)
    0x0A, 0xE2, 0x00, //   Usage (Mute)
    0x0A, 0xA2, 0x00, //   Usage (Daily)
    0x0A, 0xE9, 0x00, //   Usage (Volume Increment)
    0x0A, 0xEA, 0x00, //   Usage (Volume Decrement)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x03,       //   Input (Const, Var)
    0x0A, 0x83, 0x01, //   Usage (AL Consumer Control Configuration)
    0x0A, 0x94, 0x01, //   Usage (AL Local Machine Browser)
    0x0A, 0x86, 0x01, //   Usage (AL Spreadsheet)
    0x0A, 0x88, 0x01, //   Usage (AL Presentation App)
    0x0A, 0x8A, 0x01, //   Usage (AL Email Reader)
    0x0A, 0x92, 0x01, //   Usage (AL Calculator)
    0x0A, 0xA8, 0x02, //   Usage (unknown)
    0x0A, 0x84, 0x01, //   Usage (AL Word Processor)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x03,       //   Input (Const, Var)
    0x0A, 0x21, 0x02, //   Usage (AC Search)
    0x0A, 0x23, 0x02, //   Usage (AC Home)
    0x0A, 0x24, 0x02, //   Usage (AC Back)
    0x0A, 0x25, 0x02, //   Usage (AC Forward)
    0x0A, 0x26, 0x02, //   Usage (AC Stop)
    0x0A, 0x27, 0x02, //   Usage (AC Refresh)
    0x0A, 0x2A, 0x02, //   Usage (AC Bookmarks)
    0x0A, 0xB1, 0x02, //   Usage (unknown)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x03,       //   Input (Const, Var)
    0xC0,             // End Collection
    0x06, 0x00, 0xFF, // Usage Page (Vendor Defined)
    0x09, 0x01,       // Usage (unknown)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x04,       //   Report ID (0x04)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x09, 0x00,       //   Usage (unknown)
    0x75, 0x08,       //   Report Size (8)
    0x96, 0x07, 0x02, //   Report Count (519)
    0xB1, 0x02,       //   Feature (Var)
    0xC0,             // End Collection
    0x06, 0x00, 0xFF, // Usage Page (Vendor Defined)
    0x09, 0x01,       // Usage (unknown)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x07,       //   Report ID (0x07)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x09, 0x00,       //   Usage (unknown)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x07,       //   Report Count (7)
    0x81, 0x00,       //   Input ()
    0xC0,             // End Collection
    0x06, 0x00, 0xFF, // Usage Page (Vendor Defined)
    0x09, 0x01,       // Usage (unknown)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x05,       //   Report ID (0x05)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x09, 0x00,       //   Usage (unknown)
    0x95, 0x05,       //   Report Count (5)
    0x75, 0x08,       //   Report Size (8)
    0xB1, 0x02,       //   Feature (Var)
    0xC0              // End Collection
};

/**
 * @brief String descriptor
 */
const char *hid_string_descriptor[] = {
    (char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "SINO WEALTH",        // 1: Manufacturer
    "Gaming Mouse",       // 2: Product
    "123456",             // 3: Serials, should use chip ID
    "Gaming Mouse HID",   // 4: HID
};

/**
 * @brief Configuration descriptor
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length,
    // attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, 59, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),

    // Interface 0: Mouse (using HID_ITF_PROTOCOL_MOUSE which is 2)
    TUD_HID_DESCRIPTOR(0, 0, 2, sizeof(hid_mouse_report_desc), 0x81, 8, 1),

    // Interface 1: Keyboard (using HID_ITF_PROTOCOL_KEYBOARD which is 1)
    TUD_HID_DESCRIPTOR(1, 0, 1, sizeof(hid_keyboard_report_desc), 0x82, 8, 1)};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
  if (instance == 0) {
    return hid_mouse_report_desc;
  } else {
    return hid_keyboard_report_desc;
  }
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
static void parse_and_send_mouse(const char *line, uint32_t &recv_count,
                                 uint32_t &sent_count, uint32_t &fail_count) {
  recv_count++;

  const char *p = strchr(line, '|');
  if (!p)
    return;
  p++; // skip '|'
  while (*p == ' ')
    p++; // skip spaces

  char *next;
  long button = strtol(p, &next, 10);
  if (next == p || *next != '/')
    return;
  p = next + 1;

  long x = strtol(p, &next, 10);
  if (next == p || *next != '/')
    return;
  p = next + 1;

  long y = strtol(p, &next, 10);
  if (next == p || *next != '/')
    return;
  p = next + 1;

  long wheel = strtol(p, &next, 10);
  if (next == p)
    return;

  if (tud_mounted()) {
    static int16_t accum_x = 0;
    static int16_t accum_y = 0;
    static int8_t accum_wheel = 0;

    int16_t send_x = (int16_t)x + accum_x;
    int16_t send_y = (int16_t)y + accum_y;
    int8_t send_wheel = (int8_t)wheel + accum_wheel;

    if (tud_hid_n_ready(0)) {
      if (tud_hid_n_mouse_report(0, 0, (uint8_t)button, send_x, send_y,
                                 send_wheel, 0)) {
        sent_count++;
        accum_x = 0;
        accum_y = 0;
        accum_wheel = 0;
      } else {
        fail_count++;
        accum_x += (int16_t)x;
        accum_y += (int16_t)y;
        accum_wheel += (int8_t)wheel;
      }
    } else {
      fail_count++;
      accum_x += (int16_t)x;
      accum_y += (int16_t)y;
      accum_wheel += (int8_t)wheel;
    }
  }
}

/**
 * @brief FreeRTOS task: baca data dari Serial2 (HID reader, RX=17 TX=18),
 *        forward ke Serial default (USB/UART0) dan parse untuk TinyUSB.
 */
static void serial2_forward_task(void *arg) {
  static char line_buf[128];
  static int line_idx = 0;

  uint32_t recv_count = 0;
  uint32_t sent_count = 0;
  uint32_t fail_count = 0;
  uint32_t last_report_time = millis();

  while (true) {
    bool had_data = false;
    while (Serial2.available()) {
      had_data = true;
      char c = (char)Serial2.read();

      // Accumulate into line buffer
      if (c == '\n' || c == '\r') {
        if (line_idx > 0) {
          line_buf[line_idx] = '\0';
          parse_and_send_mouse(line_buf, recv_count, sent_count, fail_count);
          line_idx = 0;
        }
      } else {
        if (line_idx < (int)sizeof(line_buf) - 1) {
          line_buf[line_idx++] = c;
        } else {
          // buffer overflow, reset
          line_idx = 0;
        }
      }
    }

    // Print rate metrics once per second
    uint32_t now = millis();
    if (now - last_report_time >= 1000) {
      Serial.printf("[RATE] Recv: %u/sec, Sent: %u/sec, Fail: %u/sec\n",
                    (unsigned int)recv_count, (unsigned int)sent_count,
                    (unsigned int)fail_count);
      recv_count = 0;
      sent_count = 0;
      fail_count = 0;
      last_report_time = now;
    }

    if (!had_data) {
      vTaskDelay(pdMS_TO_TICKS(1)); // yield when idle
    } else {
      vTaskDelay(0); // cooperative yield when busy
    }
  }
}

/********* Application ***************/

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

  // Task untuk forward Serial2 → Serial default & parse ke TinyUSB
  xTaskCreate(serial2_forward_task, "serial2_fwd", 3072, NULL, 5, NULL);

  ESP_LOGI(TAG, "USB initialization");
  tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();

  tusb_cfg.descriptor.device = &custom_device_descriptor;
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
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
