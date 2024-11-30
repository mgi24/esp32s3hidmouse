/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "arduino.h"
#include "SPI.h"
#include "EthernetESP32.h"
#define HSPI_CS 34
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
extern "C"
{
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "errno.h"
#include "driver/gpio.h"

#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"
}
extern "C"
{
    void app_main(void);
}

/* GPIO Pin number for quit from example logic */
#define APP_QUIT_PIN GPIO_NUM_0

static const char *TAG = "example";
QueueHandle_t hid_host_event_queue;
bool user_shutdown = false;

/**
 * @brief HID Host event
 *
 * This event is used for delivering the HID Host event from callback to a task.
 */
typedef struct
{
    hid_host_device_handle_t hid_device_handle;
    hid_host_driver_event_t event;
    void *arg;
} hid_host_event_queue_t;

/**
 * @brief HID Protocol string names
 */
static const char *hid_proto_name_str[] = {
    "NONE",
    "KEYBOARD",
    "MOUSE"};

/**
 * @brief Key event
 */
typedef struct
{
    enum key_state
    {
        KEY_STATE_PRESSED = 0x00,
        KEY_STATE_RELEASED = 0x01
    } state;
    uint8_t modifier;
    uint8_t key_code;
} key_event_t;

/* Main char symbol for ENTER key */
#define KEYBOARD_ENTER_MAIN_CHAR '\r'
/* When set to 1 pressing ENTER will be extending with LineFeed during serial debug output */
#define KEYBOARD_ENTER_LF_EXTEND 1

/**
 * @brief Scancode to ascii table
 */
const uint8_t keycode2ascii[57][2] = {
    {0, 0},                                               /* HID_KEY_NO_PRESS        */
    {0, 0},                                               /* HID_KEY_ROLLOVER        */
    {0, 0},                                               /* HID_KEY_POST_FAIL       */
    {0, 0},                                               /* HID_KEY_ERROR_UNDEFINED */
    {'a', 'A'},                                           /* HID_KEY_A               */
    {'b', 'B'},                                           /* HID_KEY_B               */
    {'c', 'C'},                                           /* HID_KEY_C               */
    {'d', 'D'},                                           /* HID_KEY_D               */
    {'e', 'E'},                                           /* HID_KEY_E               */
    {'f', 'F'},                                           /* HID_KEY_F               */
    {'g', 'G'},                                           /* HID_KEY_G               */
    {'h', 'H'},                                           /* HID_KEY_H               */
    {'i', 'I'},                                           /* HID_KEY_I               */
    {'j', 'J'},                                           /* HID_KEY_J               */
    {'k', 'K'},                                           /* HID_KEY_K               */
    {'l', 'L'},                                           /* HID_KEY_L               */
    {'m', 'M'},                                           /* HID_KEY_M               */
    {'n', 'N'},                                           /* HID_KEY_N               */
    {'o', 'O'},                                           /* HID_KEY_O               */
    {'p', 'P'},                                           /* HID_KEY_P               */
    {'q', 'Q'},                                           /* HID_KEY_Q               */
    {'r', 'R'},                                           /* HID_KEY_R               */
    {'s', 'S'},                                           /* HID_KEY_S               */
    {'t', 'T'},                                           /* HID_KEY_T               */
    {'u', 'U'},                                           /* HID_KEY_U               */
    {'v', 'V'},                                           /* HID_KEY_V               */
    {'w', 'W'},                                           /* HID_KEY_W               */
    {'x', 'X'},                                           /* HID_KEY_X               */
    {'y', 'Y'},                                           /* HID_KEY_Y               */
    {'z', 'Z'},                                           /* HID_KEY_Z               */
    {'1', '!'},                                           /* HID_KEY_1               */
    {'2', '@'},                                           /* HID_KEY_2               */
    {'3', '#'},                                           /* HID_KEY_3               */
    {'4', '$'},                                           /* HID_KEY_4               */
    {'5', '%'},                                           /* HID_KEY_5               */
    {'6', '^'},                                           /* HID_KEY_6               */
    {'7', '&'},                                           /* HID_KEY_7               */
    {'8', '*'},                                           /* HID_KEY_8               */
    {'9', '('},                                           /* HID_KEY_9               */
    {'0', ')'},                                           /* HID_KEY_0               */
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR}, /* HID_KEY_ENTER           */
    {0, 0},                                               /* HID_KEY_ESC             */
    {'\b', 0},                                            /* HID_KEY_DEL             */
    {0, 0},                                               /* HID_KEY_TAB             */
    {' ', ' '},                                           /* HID_KEY_SPACE           */
    {'-', '_'},                                           /* HID_KEY_MINUS           */
    {'=', '+'},                                           /* HID_KEY_EQUAL           */
    {'[', '{'},                                           /* HID_KEY_OPEN_BRACKET    */
    {']', '}'},                                           /* HID_KEY_CLOSE_BRACKET   */
    {'\\', '|'},                                          /* HID_KEY_BACK_SLASH      */
    {'\\', '|'},
    /* HID_KEY_SHARP           */ // HOTFIX: for NonUS Keyboards repeat HID_KEY_BACK_SLASH
    {';', ':'},                   /* HID_KEY_COLON           */
    {'\'', '"'},                  /* HID_KEY_QUOTE           */
    {'`', '~'},                   /* HID_KEY_TILDE           */
    {',', '<'},                   /* HID_KEY_LESS            */
    {'.', '>'},                   /* HID_KEY_GREATER         */
    {'/', '?'}                    /* HID_KEY_SLASH           */
};

/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto)
{
    static hid_protocol_t prev_proto_output = HID_PROTOCOL_NONE;

    if (prev_proto_output != proto)
    {
        prev_proto_output = proto;
        printf("\r\n");
        if (proto == HID_PROTOCOL_MOUSE)
        {
            printf("Mouse\r\n");
        }
        else if (proto == HID_PROTOCOL_KEYBOARD)
        {
            printf("Keyboard\r\n");
        }
        else
        {
            printf("Generic\r\n");
        }
        fflush(stdout);
    }
}

/**
 * @brief HID Keyboard modifier verification for capitalization application (right or left shift)
 *
 * @param[in] modifier
 * @return true  Modifier was pressed (left or right shift)
 * @return false Modifier was not pressed (left or right shift)
 *
 */
static inline bool hid_keyboard_is_modifier_shift(uint8_t modifier)
{
    if (((modifier & HID_LEFT_SHIFT) == HID_LEFT_SHIFT) ||
        ((modifier & HID_RIGHT_SHIFT) == HID_RIGHT_SHIFT))
    {
        return true;
    }
    return false;
}

/**
 * @brief HID Keyboard get char symbol from key code
 *
 * @param[in] modifier  Keyboard modifier data
 * @param[in] key_code  Keyboard key code
 * @param[in] key_char  Pointer to key char data
 *
 * @return true  Key scancode converted successfully
 * @return false Key scancode unknown
 */
static inline bool hid_keyboard_get_char(uint8_t modifier,
                                         uint8_t key_code,
                                         unsigned char *key_char)
{
    uint8_t mod = (hid_keyboard_is_modifier_shift(modifier)) ? 1 : 0;

    if ((key_code >= HID_KEY_A) && (key_code <= HID_KEY_SLASH))
    {
        *key_char = keycode2ascii[key_code][mod];
    }
    else
    {
        // All other key pressed
        return false;
    }

    return true;
}

/**
 * @brief HID Keyboard print char symbol
 *
 * @param[in] key_char  Keyboard char to stdout
 */
static inline void hid_keyboard_print_char(unsigned int key_char)
{
    if (!!key_char)
    {
        putchar(key_char);
#if (KEYBOARD_ENTER_LF_EXTEND)
        if (KEYBOARD_ENTER_MAIN_CHAR == key_char)
        {
            putchar('\n');
        }
#endif // KEYBOARD_ENTER_LF_EXTEND
        fflush(stdout);
    }
}

/**
 * @brief Key Event. Key event with the key code, state and modifier.
 *
 * @param[in] key_event Pointer to Key Event structure
 *
 */
static void key_event_callback(key_event_t *key_event)
{
    unsigned char key_char;

    hid_print_new_device_report_header(HID_PROTOCOL_KEYBOARD);

    if (key_event->state == key_event_t::KEY_STATE_PRESSED)
    {
        if (hid_keyboard_get_char(key_event->modifier,
                                  key_event->key_code, &key_char))
        {

            hid_keyboard_print_char(key_char);
        }
    }
}

/**
 * @brief Key buffer scan code search.
 *
 * @param[in] src       Pointer to source buffer where to search
 * @param[in] key       Key scancode to search
 * @param[in] length    Size of the source buffer
 */
static inline bool key_found(const uint8_t *const src,
                             uint8_t key,
                             unsigned int length)
{
    for (unsigned int i = 0; i < length; i++)
    {
        if (src[i] == key)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(const uint8_t *const data, const int length)
{
    hid_keyboard_input_report_boot_t *kb_report = (hid_keyboard_input_report_boot_t *)data;

    if (length < sizeof(hid_keyboard_input_report_boot_t))
    {
        return;
    }

    static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};
    key_event_t key_event;

    for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++)
    {

        // key has been released verification
        if (prev_keys[i] > HID_KEY_ERROR_UNDEFINED &&
            !key_found(kb_report->key, prev_keys[i], HID_KEYBOARD_KEY_MAX))
        {
            key_event.key_code = prev_keys[i];
            key_event.modifier = 0;
            key_event.state = key_event_t::KEY_STATE_RELEASED;
            key_event_callback(&key_event);
        }

        // key has been pressed verification
        if (kb_report->key[i] > HID_KEY_ERROR_UNDEFINED &&
            !key_found(prev_keys, kb_report->key[i], HID_KEYBOARD_KEY_MAX))
        {
            key_event.key_code = kb_report->key[i];
            key_event.modifier = kb_report->modifier.val;
            key_event.state = key_event_t::KEY_STATE_PRESSED;
            key_event_callback(&key_event);
        }
    }

    memcpy(prev_keys, &kb_report->key, HID_KEYBOARD_KEY_MAX);
}

/**
 * @brief USB HID Host Mouse Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */


int smoothx = 10;
int smoothy = 10;
void sentMouse(int &btn, int &x, int &y, int &scroll, bool &mousehandled, String from)
{
    if (btn != 0 || x != 0 || y != 0 || scroll != 0 || !mousehandled)
    {
        mousehandled = true;
        Serial1.printf("2/%d/%d/%d/%d\n",
                       btn, x, y, scroll);
        btn = 0;
        x = 0;
        y = 0;
        scroll = 0;
        Serial.printf("Message from: %s \n", from.c_str());
        Serial1.flush();
    }
}


typedef struct
{
    int8_t _unknown;
    union
    {
        struct
        {
            uint8_t left_button : 1;   // Bit pertama untuk tombol kiri
            uint8_t right_button : 1;  // Bit kedua untuk tombol kanan
            uint8_t middle_button : 1; // Bit ketiga untuk tombol tengah
            uint8_t backward : 1;      // Bit keempat untuk tombol backward
            uint8_t forward : 1;       // Bit kelima untuk tombol forward
            uint8_t reserved : 3;      // Sisanya adalah bit reserved
        };
        uint8_t button; // Untuk akses byte penuh jika diperlukan
    };

    int16_t x_displacement;
    int16_t y_displacement;
    int8_t wheel;
} mouse_data;

long last_time = 0;
int maxvalx = 0;
int maxvaly = 0;
int x = 0;
int y = 0;
int btn = 0;
int scroll = 0;
bool aimmouse = false;
bool triggermouse = false;
int targetx = 0;
int targety = 0;

bool mousehandled = true;
bool targetreached = true;
void clearMouse(){
    //sometimes button is not released
    //btn = 0;
    x=0;
    y=0;
    scroll = 0;
}
static void hid_host_mouse_report_callback(const uint8_t *const data, const int length)
{
    if (length != 7)
    {
        return;
    }
    hid_print_new_device_report_header(HID_PROTOCOL_MOUSE);

    mouse_data *mdata = (mouse_data *)data;
    // for (int i = 0; i < length; i++) {
    //     Serial1.printf("0x%X ", (unsigned int)data[i]);
    // }

    // Serial1.println();
    x = mdata->x_displacement;
    y = mdata->y_displacement;
    btn = mdata->button;
    scroll = mdata->wheel;

    if (abs(mdata->x_displacement) > maxvalx)
    {
        maxvalx = abs(mdata->x_displacement);
    }

    if (abs(mdata->y_displacement) > maxvaly)
    {
        maxvaly = abs(mdata->y_displacement);
    }
    aimmouse = mdata->forward;
    triggermouse = mdata->backward;
    long current_time = millis();
    long time_diff = current_time - last_time;
    last_time = current_time;
    int fps = 1000 / time_diff;
    // Serial1.printf("X: %04d Y: %04d W: %04d |%c|%c|%c|%c|%c %ld ms %d fps %d maxX %d maxY \n",
    //        mdata->x_displacement, mdata->y_displacement, mdata->wheel,
    //        (mdata->left_button ? 'o' : ' '),
    //        (mdata->middle_button ? 'o' : ' '),
    //        (mdata->right_button ? 'o' : ' '),
    //        (mdata->forward ? 'o' : ' '),
    //        (mdata->backward ? 'o' : ' '),
    //        time_diff, fps, maxvalx, maxvaly);
    mousehandled = false;

    // if (aimmouse) {
    //     Serial1.printf("2/%d/%d/%d/%d\n",
    //                btn, x, y, scroll);

    //     Serial.printf("Target X: %d, Target Y: %d\n", targetx, targety);
    // }else{
    //     Serial1.printf("2/%d/%d/%d/%d\n",
    //                btn, x, y, scroll);
    //     Serial.printf("no trigger\n");
    // }

    // Serial1.flush();
    fflush(stdout);
}

/**
 * @brief USB HID Host Generic Interface report callback handler
 *
 * 'generic' means anything else than mouse or keyboard
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_generic_report_callback(const uint8_t *const data, const int length)
{
    hid_print_new_device_report_header(HID_PROTOCOL_NONE);
    for (int i = 0; i < length; i++)
    {
        printf("%02X", data[i]);
    }
    putchar('\r');
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                 const hid_host_interface_event_t event,
                                 void *arg)
{
    uint8_t data[64] = {0};
    size_t data_length = 0;
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

    switch (event)
    {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(hid_device_handle,
                                                                  data,
                                                                  64,
                                                                  &data_length));

        if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class)
        {
            if (HID_PROTOCOL_MOUSE == dev_params.proto)
            {
                hid_host_mouse_report_callback(data, data_length);
            }
        }
        else
        {
            hid_host_generic_report_callback(data, data_length);
        }

        break;
    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HID Device, protocol '%s' DISCONNECTED",
                 hid_proto_name_str[dev_params.proto]);
        ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
        break;
    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGI(TAG, "HID Device, protocol '%s' TRANSFER_ERROR",
                 hid_proto_name_str[dev_params.proto]);
        break;
    default:
        ESP_LOGE(TAG, "HID Device, protocol '%s' Unhandled event",
                 hid_proto_name_str[dev_params.proto]);
        break;
    }
}

/**
 * @brief USB HID Host Device event
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host Device event
 * @param[in] arg                Pointer to arguments, does not used
 */
void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                           const hid_host_driver_event_t event,
                           void *arg)
{
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

    switch (event)
    {
    case HID_HOST_DRIVER_EVENT_CONNECTED:
    {
        ESP_LOGI(TAG, "HID Device, protocol '%s' CONNECTED",
                 hid_proto_name_str[dev_params.proto]);

        const hid_host_device_config_t dev_config = {
            .callback = hid_host_interface_callback, // TO OUTPUT REPORT
            .callback_arg = NULL};

        ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
        if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class)
        {
            ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_REPORT));
            if (HID_PROTOCOL_KEYBOARD == dev_params.proto)
            {
                ESP_ERROR_CHECK(hid_class_request_set_idle(hid_device_handle, 0, 0));
            }
        }
        ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
        break;
    }

    default:
        break;
    }
}

/**
 * @brief Start USB Host install and handle common USB host library events while app pin not low
 *
 * @param[in] arg  Not used
 */
static void usb_lib_task(void *arg)
{
    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin));

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };

    ESP_ERROR_CHECK(usb_host_install(&host_config));
    xTaskNotifyGive(static_cast<TaskHandle_t>(arg));

    while (gpio_get_level(APP_QUIT_PIN) != 0)
    {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        // Release devices once all clients has deregistered
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            usb_host_device_free_all();
            ESP_LOGI(TAG, "USB Event flags: NO_CLIENTS");
        }
        // All devices were removed
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            ESP_LOGI(TAG, "USB Event flags: ALL_FREE");
        }
    }
    // App Button was pressed, trigger the flag
    user_shutdown = true;
    ESP_LOGI(TAG, "USB shutdown");
    // Clean up USB Host
    vTaskDelay(10); // Short delay to allow clients clean-up
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}

int min(int target, int input)
{
    int abstarget = abs(target);
    int absinput = abs(input);
    int result = (abstarget< absinput) ? target: input;
    if (abstarget == absinput){
        result = target;
        Serial.printf("EQUAL target: %d, input: %d", target, input);
    }
    
    return result;
}

void sentMouseTarget(int &btn, int &x, int &y, int &targetx, int &targety, int &scroll, bool &targetreached, bool &mousehandled, String from)
{
    if (!targetreached)
    {   int smoothx_sent = smoothx;
        int smoothy_sent = smoothy;
        if (targetx<0){
            smoothx_sent = -smoothx_sent;
            Serial.printf("NEGATIVE X ");
        }
        if (targety<0){
            smoothy_sent = -smoothy_sent;
            Serial.printf("NEGATIVE Y ");
        }
        Serial.printf("Target X: %d, Target Y: %d ", targetx, targety);
        int sentx = min(targetx, (x + min(targetx, smoothx_sent)));
        int senty = min(targety, (y + min(targety, smoothy_sent)));
        if (sentx == targetx && senty == targety)
        {
            targetreached = true;
            Serial.printf("TARGET REACHED\n");
        }
        targetx -= sentx;
        targety -= senty;
        
        if (!mousehandled)
        {
            mousehandled = true;
            Serial1.printf("2/%d/%d/%d/%d\n",
                           btn, sentx, senty, scroll);
            Serial.printf("Target X: %d, Target Y: %d SentX: %d SentY: %d via mousehandle\n", targetx, targety, sentx, senty);
        }
        else if (mousehandled)
        {
            Serial1.printf("2/%d/%d/%d/%d\n",
                           btn, sentx, senty, scroll);
            Serial.printf("Target X: %d, Target Y: %d SentX: %d SentY: %d via auto       \n", targetx, targety, sentx, senty);
            vTaskDelay(8);
        }
        clearMouse();
    }
    else{
        if(!mousehandled){
            mousehandled = true;
            Serial1.printf("2/%d/%d/%d/%d\n",
                            btn, x, y, scroll);
            targetx-=x;
            targety-=y;
            targetreached = false;
            Serial.printf("TARGET OVERSHOOT X: %d, Y: %d\n", x, y);
            clearMouse();
            vTaskDelay(8);
        }
        
        
    }
}

void triggerMouse(int &btn, int &targetx, int &targety, int &scroll){
    btn = 0;
    Serial1.printf("2/%d/%d/%d/%d\n",
                            btn, targetx, targety, scroll);
    targetx = 0;
    targety = 0;
    targetreached = true;
    vTaskDelay(5);
    Serial1.printf("2/%d/%d/%d/%d\n",
                            1, 0, 0, 0);
    Serial.printf("TRIGGER LAUNCHED!\n");
}
void ethernet_task(void *pvParameters)
{
    EthernetServer *server = (EthernetServer *)pvParameters;

    while (1)
    {
        EthernetClient client = server->available();
        if (client)
        {
            while (client.connected())
            {
                
                
                if (client.available())
                {   
                    String message = client.readStringUntil('\n');
                    int separatorIndex = message.indexOf('/');
                    if (separatorIndex != -1)
                    {
                        targetx = message.substring(0, separatorIndex).toInt();
                        targety = message.substring(separatorIndex + 1).toInt();
                        Serial.printf("TARGET AQC, X: %d, Y: %d\n", targetx, targety);
                        targetreached = false;
                    }
                    if(aimmouse){
                        sentMouseTarget(btn, x, y, targetx, targety, scroll, targetreached, mousehandled, "from message");
                    }
                    if(triggermouse){
                        triggerMouse(btn, targetx, targety, scroll);
                    }
                    else{
                        sentMouse(btn, x, y, scroll, mousehandled, "from message");
                    }
                    
                }
                else{
                    if(aimmouse){
                        sentMouseTarget(btn, x, y, targetx, targety, scroll, targetreached, mousehandled, "from message");
                    }
                    else{
                        sentMouse(btn, x, y, scroll, mousehandled, "NO MSG");
                    }
                    
                }
                
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            // Koneksi client ditutup
            client.stop();
        }
        sentMouse(btn, x, y, scroll, mousehandled, "from ethernet");
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    vTaskDelete(NULL);
}

void hid_host_task(void *pvParameters)
{
    hid_host_event_queue_t evt_queue;
    // Create queue
    hid_host_event_queue = xQueueCreate(10, sizeof(hid_host_event_queue_t));

    // Wait queue
    while (!user_shutdown)
    {
        if (xQueueReceive(hid_host_event_queue, &evt_queue, pdMS_TO_TICKS(50)))
        {
            hid_host_device_event(evt_queue.hid_device_handle, // TO OUTPUT
                                  evt_queue.event,
                                  evt_queue.arg);
        }
    }

    xQueueReset(hid_host_event_queue);
    vQueueDelete(hid_host_event_queue);
    vTaskDelete(NULL);
}

/**
 * @brief HID Host Device callback
 *
 * Puts new HID Device event to the queue
 *
 * @param[in] hid_device_handle HID Device handle
 * @param[in] event             HID Device event
 * @param[in] arg               Not used
 */
void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                              const hid_host_driver_event_t event,
                              void *arg)
{
    const hid_host_event_queue_t evt_queue = {
        .hid_device_handle = hid_device_handle,
        .event = event,
        .arg = arg};
    xQueueSend(hid_host_event_queue, &evt_queue, 0);
}

EthernetServer server(80); // set HTTP server port
#define RX_PIN1 16
#define TX_PIN1 17
#define RX_PIN2 3
#define TX_PIN2 2
void app_main(void)
{
    BaseType_t task_created;
    ESP_LOGI(TAG, "HID Host example");
    Serial1.begin(115200, SERIAL_8N1, RX_PIN1, TX_PIN1);
    Serial.begin(115200, SERIAL_8N1, RX_PIN2, TX_PIN2);
    pinMode(15, OUTPUT);
    digitalWrite(15, HIGH);
    Ethernet.init(HSPI_CS);
    // Memulai Ethernet
    if (Ethernet.begin(mac) == 0)
    {
        Serial.println("Gagal memulai Ethernet. Periksa koneksi HSPI.");
    }
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
        Serial.println("Tidak ada perangkat Ethernet yang terdeteksi.");
    }
    else if (Ethernet.hardwareStatus() == EthernetW5100)
    {
        Serial.println("W5100 terdeteksi.");
    }
    // Print the IP address to the serial monitor
    Serial.print("IP Address: ");
    Serial.println(Ethernet.localIP());
    server.begin();
    Serial.println("Hello World");
    // Check and print the CPU frequency

    /*
     * Create usb_lib_task to:
     * - initialize USB Host library
     * - Handle USB Host events while APP pin in in HIGH state
     */
    task_created = xTaskCreatePinnedToCore(usb_lib_task,
                                           "usb_events",
                                           4096,
                                           xTaskGetCurrentTaskHandle(),
                                           2, NULL, 0);
    assert(task_created == pdTRUE);

    // Wait for notification from usb_lib_task to proceed
    ulTaskNotifyTake(false, 1000);

    /*
     * HID host driver configuration
     * - create background task for handling low level event inside the HID driver
     * - provide the device callback to get new HID Device connection event
     */
    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_device_callback,
        .callback_arg = NULL};

    ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

    // Task is working until the devices are gone (while 'user_shutdown' if false)
    user_shutdown = false;

    /*
     * Create HID Host task process for handle events
     * IMPORTANT: Task is necessary here while there is no possibility to interact
     * with USB device from the callback.
     */
    task_created = xTaskCreate(&hid_host_task, "hid_task", 4 * 1024, NULL, 2, NULL);
    assert(task_created == pdTRUE);
    // Create Ethernet task to handle Ethernet data
    task_created = xTaskCreate(&ethernet_task, "ethernet_task", 4 * 1024, &server, 2, NULL);
    assert(task_created == pdTRUE);
}