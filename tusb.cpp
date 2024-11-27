/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "arduino.h"
extern "C" {
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
}
extern "C" {
    void app_main(void);
}


#define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default
static const char *TAG = "example";

/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    //TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD) ),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE) )
};

/**
 * @brief String descriptor
 */
const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "INSTANT",             // 1: Manufacturer
    "USB GAMING MOUSE",      // 2: Product
    "",              // 3: Serials, should use chip ID
    "INSTANT",  // 4: HID
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
}

/********* Application ***************/

typedef enum {
    MOUSE_DIR_RIGHT,
    MOUSE_DIR_DOWN,
    MOUSE_DIR_LEFT,
    MOUSE_DIR_UP,
    MOUSE_DIR_MAX,
} mouse_dir_t;




// Define postfix increment operator for mouse_dir_t
mouse_dir_t operator++(mouse_dir_t &dir, int) {
    mouse_dir_t old_dir = dir;
    dir = static_cast<mouse_dir_t>((static_cast<int>(dir) + 1) % MOUSE_DIR_MAX);
    return old_dir;
}



static void app_send_hid_demo(void)
{
    // Keyboard output: Send key 'a/A' pressed and released
    // ESP_LOGI(TAG, "Sending Keyboard report");
    // uint8_t keycode[6] = {HID_KEY_A};
    // tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
    // vTaskDelay(pdMS_TO_TICKS(50));
    //tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);

    // Mouse output: Move mouse cursor in square trajectorya
    ESP_LOGI(TAG, "Sending Mouse report");

        // Get the next x and y delta in the draw square pattern
    tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, 0x00, 127, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
}

const tusb_desc_device_t my_device_descriptor = {
    .bLength = sizeof(my_device_descriptor),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0110,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x30FA, // This is Espressif VID. This needs to be changed according to Users / Customers
    .idProduct = 0x1701,
    .bcdDevice = 0x100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x00,
    .bNumConfigurations = 0x01
};



void process_input(int id, int btn, int delta_x, int delta_y, int scroll, int pan) {
    uint8_t buffer[8];
    buffer[0] = static_cast<uint8_t>(id);
    buffer[1] = static_cast<uint8_t>(btn);
    buffer[2] = static_cast<uint8_t>(delta_x & 0xFF);
    buffer[3] = static_cast<uint8_t>((delta_x >> 8) & 0xFF);
    buffer[4] = static_cast<uint8_t>(delta_y & 0xFF);
    buffer[5] = static_cast<uint8_t>((delta_y >> 8) & 0xFF);
    buffer[6] = static_cast<uint8_t>(scroll);
    buffer[7] = static_cast<uint8_t>(pan);
    

    Serial.print("Buffer: ");
    for (int i = 0; i < sizeof(buffer); i++) {
        Serial.printf("%02X ", buffer[i]);
    }
    
}
#define RX2 15
#define TX2 16
static uint8_t mouse_buttons = 0;
void app_main(void)
{   Serial2.begin(115200, SERIAL_8N1, RX2, TX2);
    Serial.begin(115200);

    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &my_device_descriptor,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
    long lasttime = 0;
    while (1) {
        
        if (tud_mounted()) {
            if (Serial.available() > 0) {
                String input = Serial.readStringUntil('\n');
                int id, btn, delta_x, delta_y, scroll;
                if (sscanf(input.c_str(), "%d/%d/%d/%d/%d", &id, &btn, &delta_x, &delta_y, &scroll) == 5) {
                    bool result = tud_hid_abs_mouse_report(id, btn, delta_x, delta_y, scroll, 0);
                    process_input(id, btn, delta_x, delta_y, scroll, 0);
                    Serial.print("result ");
                    Serial.println(result);
                } else {
                    Serial.println("Invalid input format. Expected format: btn/deltax/deltay/scroll");
                }
                
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
