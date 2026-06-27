# ESP32 TinyUSB HID Mouse (16-bit Resolution)

Projek ini mengimplementasikan USB HID Mouse + Keyboard menggunakan library TinyUSB di ESP32 (S2/S3/P4), dengan modifikasi resolusi koordinat mouse relatif menjadi **16-bit** (bukan bawaan standar 8-bit) dan fitur passthrough Serial2.

| Supported Targets | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | -------- | -------- | -------- |

---

## 1. Fitur Utama & Pembacaan Serial dari Pin (Serial Passthrough)

Di dalam file source utama ([tusb_mouse_ethernet.cpp](file:///d:/CODING/tusb_hid/main/tusb_mouse_ethernet.cpp) atau [tusb_mouse.cpp](file:///d:/CODING/tusb_hid/main/tusb_mouse.cpp)), terdapat sistem pembacaan data serial dari perangkat HID reader eksternal yang di-forward ke PC sebagai berikut:

* **Inisialisasi Serial2:**
  Diinisialisasi di fungsi `app_main()` dengan baud rate `115200` pada pin berikut:
  * **RX Pin:** GPIO 17
  * **TX Pin:** GPIO 18 (sisi ESP32 terhubung ke RX reader/TX eksternal)
  ```cpp
  Serial2.begin(115200, SERIAL_8N1, 17, 18);
  ```

* **FreeRTOS Passthrough Task:**
  Pembacaan dilakukan secara asinkron menggunakan FreeRTOS task `serial2_forward_task`:
  ```cpp
  static void serial2_forward_task(void *arg) {
    while (true) {
      if (Serial2.available()) {
        while (Serial2.available()) {
          char c = (char)Serial2.read();
          Serial.write(c); // Forward ke default Serial (USB/UART0)
        }
      }
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
  ```
  Task ini membaca data biner/karakter yang diterima di Pin 17 (`Serial2`) lalu langsung menulisnya (`Serial.write`) ke PC lewat port serial utama (USB/UART0) untuk monitoring atau parsing lebih lanjut.

---

## 2. Riwayat Perubahan Koordinat Mouse (Upgrade ke 16-bit)

Untuk mengatasi limitasi default TinyUSB di mana koordinat Mouse relatif hanya mendukung nilai dari `-127` sampai `127` (8-bit), telah dilakukan modifikasi di tingkat komponen TinyUSB agar mendukung resolusi **16-bit** (`-32767` sampai `32767`) layaknya mouse gaming modern:

### A. Modifikasi File [hid_device.h](file:///d:/CODING/tusb_hid/managed_components/espressif__tinyusb/src/class/hid/hid_device.h)
* **Mengubah Descriptor Template Mouse (`TUD_HID_REPORT_DESC_MOUSE`):**
  Mengganti makro logical min/max 8-bit menjadi makro multi-byte 16-bit (`_N`) dan mengubah ukuran report menjadi 16 bit.
  ```c
  HID_LOGICAL_MIN_N(0x8001, 2), // Mengubah min ke -32767 (2 byte)
  HID_LOGICAL_MAX_N(0x7fff, 2), // Mengubah max ke 32767 (2 byte)
  HID_REPORT_COUNT(2),
  HID_REPORT_SIZE(16),          // Mengubah ukuran report koordinat X dan Y menjadi 16-bit
  ```
* **Mengubah Signature API Helper:**
  Mengubah tipe parameter `x` dan `y` dari `int8_t` menjadi `int16_t` pada fungsi `tud_hid_n_mouse_report` dan `tud_hid_mouse_report`.

### B. Modifikasi File [hid.h](file:///d:/CODING/tusb_hid/managed_components/espressif__tinyusb/src/class/hid/hid.h)
* **Mengubah Struct Mouse Report (`hid_mouse_report_t`):**
  Mengubah tipe data variabel `x` dan `y` dari `int8_t` menjadi `int16_t`:
  ```c
  typedef struct TU_ATTR_PACKED {
    uint8_t buttons;
    int16_t x;       // Resolusi 16-bit
    int16_t y;       // Resolusi 16-bit
    int8_t wheel;
    int8_t pan;
  } hid_mouse_report_t;
  ```

### C. Modifikasi File [hid_device.c](file:///d:/CODING/tusb_hid/managed_components/espressif__tinyusb/src/class/hid/hid_device.c)
* **Mengubah Implementasi Fungsi:**
  Menyesuaikan argumen parameter `x` dan `y` pada implementasi `tud_hid_n_mouse_report()` dari `int8_t` menjadi `int16_t`.

### D. Modifikasi File Source Utama ([tusb_mouse_ethernet.cpp](file:///d:/CODING/tusb_hid/main/tusb_mouse_ethernet.cpp) dan [tusb_mouse.cpp](file:///d:/CODING/tusb_hid/main/tusb_mouse.cpp))
* **Menyesuaikan Variabel Demo:**
  Mengubah tipe data penampung delta `delta_x` dan `delta_y` di fungsi demo `app_send_hid_demo()` ke `int16_t` agar data presisi tidak terpotong (truncate) saat dikirim ke host PC.

---

## 3. Skema Koneksi Pin ESP32 ke Perangkat HID Reader Eksternal

| ESP32 Pin | Perangkat Eksternal | Keterangan |
| :--- | :--- | :--- |
| **GPIO 17 (RX2)** | TX Device | Menerima data serial dari perangkat HID reader |
| **GPIO 18 (TX2)** | RX Device | Mengirim data ke perangkat (opsional/passthrough) |
| **GND** | GND | Ground Bersama |

---

## 4. Varian Projek & Cara Mengubah Varian

Projek ini memiliki dua varian file source utama:

1. **Varian dengan Ethernet ([tusb_mouse_ethernet.cpp](file:///d:/CODING/tusb_hid/main/tusb_mouse_ethernet.cpp))**:
   Mengaktifkan USB HID Mouse + Keyboard sekaligus menginisialisasi modul Ethernet (W5500/W5100).
2. **Varian tanpa Ethernet ([tusb_mouse.cpp](file:///d:/CODING/tusb_hid/main/tusb_mouse.cpp))**:
   Hanya mengaktifkan USB HID Mouse + Keyboard tanpa menginisialisasi modul Ethernet.

### Cara Mengganti Varian yang Di-build:
Untuk mengganti varian yang dicompile, edit file [main/CMakeLists.txt](file:///d:/CODING/tusb_hid/main/CMakeLists.txt) pada bagian `SRCS`.

* Jika ingin menggunakan varian **dengan Ethernet**:
  ```cmake
  idf_component_register(
      SRCS "tusb_mouse_ethernet.cpp"
      INCLUDE_DIRS "."
      PRIV_REQUIRES esp_driver_gpio Ethernet
  )
  ```

* Jika ingin menggunakan varian **tanpa Ethernet**:
  ```cmake
  idf_component_register(
      SRCS "tusb_mouse.cpp"
      INCLUDE_DIRS "."
      PRIV_REQUIRES esp_driver_gpio Ethernet
  )
  ```
