/**
 * ╔═══════════════════════════════════════════════════════════════╗
 * ║  ESP32S3_USB_DAC.ino — Main Sketch                           ║
 * ║  ESP32-S3 DevKit-C + PCM5102A + Polyphase FIR Upsampler     ║
 * ║  USB Audio → 8× FIR → 384 kHz I2S → BLE Control            ║
 * ║  Author: Danny Liu  |  v2.0.0                                ║
 * ╚═══════════════════════════════════════════════════════════════╝
 *
 * ─── ARDUINO IDE BOARD SETTINGS (ALL REQUIRED) ──────────────────
 *   Board              : ESP32S3 Dev Module
 *   USB Mode           : USB-OTG (TinyUSB)   ← enables USB Audio UAC2
 *   USB CDC On Boot    : Enabled              ← keeps Serial working
 *   CPU Frequency      : 240 MHz              ← FIR needs full clock
 *   Flash Size         : Match your board (4 MB or 8 MB)
 *   Partition Scheme   : Default 4MB with spiffs
 *   PSRAM              : OPI PSRAM (if your board has PSRAM)
 *   Arduino Runs On    : Core 1               ← FIR task on Core 0
 *
 * ─── PHYSICAL WIRING ─────────────────────────────────────────────
 *   See config.h for full GPIO table and how to change any pin.
 *
 *   ESP32-S3       PCM5102A Module     Notes
 *   ──────────────────────────────────────────────────────────────
 *   GPIO 12   →    BCK                 I2S Bit Clock
 *   GPIO 11   →    LRCK (WS)           Word Select / LR clock
 *   GPIO 13   →    DIN                 Serial Data
 *   GPIO 10   →    XSMT               Software mute control
 *   3.3 V     →    VCC
 *   GND       →    GND, FMT, DEMP     FMT=LOW → I2S format
 *   NC/GND    →    SCK                Internal PLL (no ext clock needed)
 *
 *   PC USB cable → ESP32-S3 USB-OTG port (near GPIO 19/20)
 *   Flash cable  → ESP32-S3 UART port    (works simultaneously)
 *
 * ─── DUAL-CORE TASK MAP ─────────────────────────────────────────
 *   Core 0:  fir_task()      — polyphase FIR upsampler, I2S DMA write
 *   Core 1:  Arduino loop()  — USB audio receive, BLE, Serial AT parser
 *
 * ─── UPSAMPLING SIGNAL CHAIN ─────────────────────────────────────
 *   PC → USB Full Speed → 48 kHz / 16-bit PCM
 *     → fir_feed() queue → Core 0 fir_task()
 *     → polyphase FIR (128 taps, Kaiser β=7.0)
 *     → 384 kHz / 32-bit PCM → I2S DMA → PCM5102A → audio output
 *
 * ─── REQUIRED LIBRARIES (all in ESP32 Arduino core ≥ 2.0.11) ────
 *   USB.h, esp32-hal-tinyusb.h   — USB Audio UAC2 + CDC
 *   BLEDevice, BLEServer, etc.   — BLE NUS service
 *   Preferences                  — NVS persistent storage
 *   driver/i2s_std.h             — ESP-IDF I2S v5 driver
 *   freertos/queue.h             — inter-core audio queue
 *
 * ─── FILE STRUCTURE ─────────────────────────────────────────────
 *   ESP32S3_USB_DAC.ino   ← You are here
 *   config.h              ← All GPIOs, FIR constants, tunable params
 *   i2s_dac.h             ← I2S driver (16-bit and 32-bit modes)
 *   fir_upsample.h        ← Polyphase FIR upsampler (Core 0 task)
 *   usb_audio.h           ← USB Audio UAC2 callbacks → FIR queue
 *   ble_uart.h            ← BLE NUS service
 *   at_handler.h          ← AT command processor + debugPrint()
 */

// ─── Include order matters ────────────────────────────────────
#include "config.h"       // 1. Constants (no deps)
#include "i2s_dac.h"      // 2. I2S driver
#include "fir_upsample.h" // 3. FIR upsampler (uses i2s_dac.h)
#include "usb_audio.h"    // 4. USB callbacks (uses fir_upsample.h)
#include "ble_uart.h"     // 5. BLE service
#include "at_handler.h"   // 6. AT commands (uses all above)
#include <Preferences.h>

// ═══════════════════════════════════════════════════════════════
// GLOBAL STATE  (shared between at_handler.h and the loop)
// ═══════════════════════════════════════════════════════════════

String      g_bt_name       = BLE_DEFAULT_NAME;
bool        g_mute_state    = false;
uint32_t    g_sample_rate   = USB_AUDIO_SAMPLE_RATE;
String      g_audio_source  = "USB";
Preferences g_prefs;

// ─── Serial AT command line buffer ───────────────────────────
static String   s_at_buf;

// ─── Periodic task timers ─────────────────────────────────────
static uint32_t s_last_status_ms = 0;
static uint32_t s_last_idle_ms   = 0;

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
    // ── Serial (USB CDC) ──────────────────────────────────────
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1500);

    Serial.println();
    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║  ESP32-S3 USB DAC  v" FIRMWARE_VERSION "           ║");
    Serial.println("║  + Polyphase FIR Upsampler           ║");
    Serial.println("║  Author: Danny Liu                   ║");
    Serial.println("╚══════════════════════════════════════╝");
    Serial.println();

    // ── NVS: restore persisted settings ──────────────────────
    g_prefs.begin(NVS_NAMESPACE, false);
    g_bt_name         = g_prefs.getString(NVS_KEY_BT_NAME, BLE_DEFAULT_NAME);
    g_mute_state      = g_prefs.getBool(NVS_KEY_MUTE, false);
    // Restore saved upsampling ratio; default to FIR_DEFAULT_RATIO
    g_upsample_ratio  = g_prefs.getUChar(NVS_KEY_UPSAMPLE, FIR_DEFAULT_RATIO);

    Serial.printf("[NVS]  BT Name  : %s\n", g_bt_name.c_str());
    Serial.printf("[NVS]  Mute     : %s\n", g_mute_state ? "ON" : "OFF");
    Serial.printf("[NVS]  Upsample : ×%d\n", g_upsample_ratio);

    // ── I2S + PCM5102A ────────────────────────────────────────
    // i2s_dac_init() sets up the peripheral at USB_AUDIO_SAMPLE_RATE
    // (48 kHz). The FIR task will reconfigure to 384 kHz on first run.
    Serial.println("[I2S]  Initialising PCM5102A...");
    i2s_dac_init();
    i2s_dac_set_mute(g_mute_state);
    Serial.printf("[I2S]  BCK=GPIO%d  WS=GPIO%d  DOUT=GPIO%d  MUTE=GPIO%d\n",
                  PIN_I2S_BCK, PIN_I2S_WS, PIN_I2S_DOUT, PIN_PCM_MUTE);

    // ── FIR Upsampler (Core 0 task + inter-core queue) ────────
    Serial.printf("[FIR]  Init upsampler ×%d (USB %d Hz → %u Hz)\n",
                  g_upsample_ratio,
                  USB_AUDIO_SAMPLE_RATE,
                  (uint32_t)USB_AUDIO_SAMPLE_RATE * g_upsample_ratio);
    Serial.printf("[FIR]  Filter: %d taps (%d phases × %d), β=%.1f\n",
                  FIR_MAX_L * FIR_MAX_K, FIR_MAX_L, FIR_MAX_K, FIR_KAISER_BETA);
    fir_upsample_init(g_upsample_ratio);

    // ── USB Audio ─────────────────────────────────────────────
    Serial.println("[USB]  Starting USB Audio Class 2.0...");
    usb_audio_init();
    Serial.println("[USB]  Connect USB-OTG cable to PC. Device: 'ESP32S3 USB DAC'");

    // ── BLE UART ──────────────────────────────────────────────
    Serial.printf("[BLE]  Advertising as: %s\n", g_bt_name.c_str());
    ble_init(g_bt_name);
    Serial.println("[BLE]  Open the PWA and connect.");

    // ── Boot complete ─────────────────────────────────────────
    Serial.println();
    Serial.printf("[READY] Type AT+HELP for commands. Core 0 = FIR, Core 1 = USB/BLE\n");
    Serial.println("────────────────────────────────────────");
}

// ═══════════════════════════════════════════════════════════════
// LOOP  (runs on Core 1 — same core as USB and BLE)
// ═══════════════════════════════════════════════════════════════

void loop() {
    uint32_t now = millis();

    // ── 1. Serial AT command parser ───────────────────────────
    // Accumulate characters until CR or LF, then dispatch.
    // Test: open Serial Monitor (CR+LF line ending), type AT, Enter.
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (s_at_buf.length() > 0) {
                Serial.print("> ");
                Serial.println(s_at_buf);
                processATCommand(s_at_buf);
                s_at_buf = "";
            }
        } else if (c >= 0x20 && s_at_buf.length() < 256) {
            s_at_buf += c;
        }
    }

    // ── 2. Periodic BLE status push to Android app ────────────
    // JSON format parsed by the PWA to update live UI.
    if (g_ble_connected &&
        (now - s_last_status_ms >= BLE_STATUS_INTERVAL_MS)) {

        s_last_status_ms = now;

        bool streaming = usb_audio_is_streaming();
        usb_audio_reset_active();

        // Update source label
        if (!streaming) {
            if (now - s_last_idle_ms > 10000) g_audio_source = "Idle";
        } else {
            g_audio_source = "USB";
            s_last_idle_ms = now;
        }

        uint32_t out_rate = (g_upsample_ratio > 1)
                          ? g_sample_rate * g_upsample_ratio
                          : g_sample_rate;

        String status = String("{")
            + "\"source\":\""     + g_audio_source                         + "\","
            + "\"inRate\":"       + g_sample_rate                           + ","
            + "\"outRate\":"      + out_rate                                + ","
            + "\"upsample\":"     + g_upsample_ratio                        + ","
            + "\"mute\":"         + (g_mute_state ? "true" : "false")       + ","
            + "\"usbActive\":"    + (streaming ? "true" : "false")          + ","
            + "\"btName\":\""     + g_bt_name                               + "\""
            + "}";

        ble_send(status);
    }

    // ── 3. Yield — keeps USB stack and BLE stack healthy ──────
    // The FIR DMA and TinyUSB run in separate FreeRTOS tasks.
    // A small delay here yields the Core 1 time-slice back to
    // the scheduler so those tasks get CPU time.
    delay(5);
}
