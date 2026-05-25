/**
 * at_handler.h — AT Command Processor
 * ─────────────────────────────────────────────────────────────
 * Parses and executes AT commands received from either the PC
 * serial terminal or the Android BLE app.
 *
 * v2.0: Added AT+UPSAMPLE commands for FIR interpolation control.
 *
 * COMMAND FORMAT:
 *   AT              — test (returns OK)
 *   AT+CMD          — action command
 *   AT+CMD?         — query (returns current value)
 *   AT+CMD=value    — set command
 *
 * ADDING YOUR OWN COMMAND:
 *   1. Add an else-if block in processATCommand() below.
 *   2. Add a line to AT_HELP_TEXT.
 *   3. If persistence is needed, add an NVS key in config.h.
 *
 * Author: Danny Liu  |  v2.0.0
 */

#pragma once
#include <Arduino.h>
#include "config.h"
#include "i2s_dac.h"
#include "ble_uart.h"
#include "fir_upsample.h"
#include <Preferences.h>

// ─── External globals (owned by main .ino) ────────────────────
extern String      g_bt_name;
extern bool        g_mute_state;
extern uint32_t    g_sample_rate;
extern String      g_audio_source;
extern bool        g_usb_audio_active;
extern Preferences g_prefs;

// ─── Unified output: Serial + BLE ─────────────────────────────
void debugPrint(const String& msg) {
    Serial.println(msg);
    ble_send(msg + "\r\n");
}

// ─── Help text (also displayed verbatim in the Android PWA) ───
static const char AT_HELP_TEXT[] =
    "╔════════════════════════════════════╗\r\n"
    "║  ESP32S3-DAC  AT Command List v2   ║\r\n"
    "╚════════════════════════════════════╝\r\n"
    "AT                   Test — returns OK\r\n"
    "AT+HELP              This help listing\r\n"
    "AT+INFO              Firmware & hardware info\r\n"
    "AT+STATUS            Live audio status (JSON)\r\n"
    "─── Audio & DAC ────────────────────\r\n"
    "AT+MUTE=ON           Mute analogue output (PCM5102A XSMT)\r\n"
    "AT+MUTE=OFF          Unmute analogue output\r\n"
    "AT+MUTE?             Query mute state\r\n"
    "AT+RATE?             Query USB audio sample rate (Hz)\r\n"
    "AT+SOURCE?           Query audio source label\r\n"
    "─── FIR Upsampler ──────────────────\r\n"
    "AT+UPSAMPLE=8X       Enable 8× FIR  (48→384 kHz, 96→768*)\r\n"
    "AT+UPSAMPLE=4X       Enable 4× FIR  (48→192 kHz, 96→384 kHz)\r\n"
    "AT+UPSAMPLE=OFF      Passthrough (no upsampling, 16-bit I2S)\r\n"
    "AT+UPSAMPLE?         Query current upsampling ratio\r\n"
    "  *768 kHz exceeds PCM5102A limit; use 4X with 96 kHz input\r\n"
    "─── BLE ────────────────────────────\r\n"
    "AT+BTNAME=<n>        Set BLE device name (restart to apply)\r\n"
    "AT+BTNAME?           Query BLE device name\r\n"
    "AT+BLEOFF            Stop BLE advertising\r\n"
    "AT+BLEON             Restart BLE advertising\r\n"
    "─── System ─────────────────────────\r\n"
    "AT+RESTART           Reboot the ESP32-S3\r\n"
    "AT+FACTORY           Factory reset (clear NVS, reboot)\r\n"
    "────────────────────────────────────\r\n"
    "Commands terminated by CR+LF (\\r\\n)\r\n";

// ─── Helper: upsample ratio → display string ──────────────────
static String ratio_str(uint8_t r) {
    if (r == 1) return "OFF (passthrough)";
    return String(r) + "X (" +
           String(USB_AUDIO_SAMPLE_RATE / 1000) + "kHz → " +
           String((uint32_t)USB_AUDIO_SAMPLE_RATE * r / 1000) + "kHz)";
}

// ─── Main command dispatcher ──────────────────────────────────
void processATCommand(const String& raw) {
    String cmd = raw;
    cmd.trim();
    String cu = cmd;
    cu.toUpperCase();

    String resp;

    // ── AT ────────────────────────────────────────────────────
    if (cu == "AT") {
        resp = "OK";
    }

    // ── AT+HELP ───────────────────────────────────────────────
    else if (cu == "AT+HELP") {
        resp = AT_HELP_TEXT;
    }

    // ── AT+INFO ───────────────────────────────────────────────
    else if (cu == "AT+INFO") {
        resp = String("Model      : ") + DEVICE_MODEL + "\r\n"
             + "Firmware   : " + FIRMWARE_VERSION        + "\r\n"
             + "Author     : " + DEVICE_MANUF            + "\r\n"
             + "DAC IC     : PCM5102A via I2S\r\n"
             + "BCK GPIO   : " + PIN_I2S_BCK             + "\r\n"
             + "WS  GPIO   : " + PIN_I2S_WS              + "\r\n"
             + "DOUT GPIO  : " + PIN_I2S_DOUT            + "\r\n"
             + "MUTE GPIO  : " + PIN_PCM_MUTE            + "\r\n"
             + "BLE Name   : " + g_bt_name               + "\r\n"
             + "USB Rate   : " + USB_AUDIO_SAMPLE_RATE   + " Hz\r\n"
             + "FIR Ratio  : " + ratio_str(g_upsample_ratio) + "\r\n"
             + "FIR Taps   : " + String(FIR_MAX_L * FIR_MAX_K)
                               + " (" + FIR_MAX_L + " phases × " + FIR_MAX_K + ")\r\n"
             + "FIR Kaiser β: " + String(FIR_KAISER_BETA, 1) + "\r\n"
             + "FIR Core   : 0  (USB/BLE on Core 1)\r\n"
             + "OK";
    }

    // ── AT+STATUS ─────────────────────────────────────────────
    else if (cu == "AT+STATUS") {
        uint32_t out_rate = (g_upsample_ratio > 1)
                          ? (g_sample_rate * g_upsample_ratio)
                          : g_sample_rate;
        resp = String("{")
             + "\"source\":\""     + g_audio_source                       + "\","
             + "\"inRate\":"       + g_sample_rate                         + ","
             + "\"outRate\":"      + out_rate                              + ","
             + "\"upsample\":"     + g_upsample_ratio                      + ","
             + "\"upsampleStr\":\"" + ratio_str(g_upsample_ratio)          + "\","
             + "\"mute\":"         + (g_mute_state ? "true" : "false")     + ","
             + "\"usbActive\":"    + (g_usb_audio_active ? "true" : "false") + ","
             + "\"bleConn\":"      + (g_ble_connected ? "true" : "false")  + ","
             + "\"btName\":\""     + g_bt_name                             + "\""
             + "}";
    }

    // ── AT+MUTE ───────────────────────────────────────────────
    else if (cu == "AT+MUTE=ON") {
        g_mute_state = true;
        i2s_dac_set_mute(true);
        g_prefs.putBool(NVS_KEY_MUTE, true);
        resp = "Muted\r\nOK";
    }
    else if (cu == "AT+MUTE=OFF") {
        g_mute_state = false;
        i2s_dac_set_mute(false);
        g_prefs.putBool(NVS_KEY_MUTE, false);
        resp = "Unmuted\r\nOK";
    }
    else if (cu == "AT+MUTE?") {
        resp = String("MUTE:") + (g_mute_state ? "ON" : "OFF") + "\r\nOK";
    }

    // ── AT+UPSAMPLE= ─────────────────────────────────────────
    // The ratio change is written to g_upsample_ratio.
    // The FIR task on Core 0 picks it up on the next packet,
    // rebuilds the polyphase matrix, and reconfigures I2S — all
    // without interrupting the USB receive path.
    else if (cu == "AT+UPSAMPLE=8X" || cu == "AT+UPSAMPLE=8") {
        fir_set_ratio(8);
        g_prefs.putUChar(NVS_KEY_UPSAMPLE, 8);
        resp = "Upsampling: 8× → " +
               String((uint32_t)USB_AUDIO_SAMPLE_RATE * 8 / 1000) +
               " kHz\r\nOK";
    }
    else if (cu == "AT+UPSAMPLE=4X" || cu == "AT+UPSAMPLE=4") {
        fir_set_ratio(4);
        g_prefs.putUChar(NVS_KEY_UPSAMPLE, 4);
        resp = "Upsampling: 4× → " +
               String((uint32_t)USB_AUDIO_SAMPLE_RATE * 4 / 1000) +
               " kHz\r\nOK";
    }
    else if (cu == "AT+UPSAMPLE=OFF" || cu == "AT+UPSAMPLE=1") {
        fir_set_ratio(1);
        g_prefs.putUChar(NVS_KEY_UPSAMPLE, 1);
        resp = "Upsampling: OFF (passthrough " +
               String(USB_AUDIO_SAMPLE_RATE / 1000) + " kHz)\r\nOK";
    }
    else if (cu == "AT+UPSAMPLE?") {
        resp = "UPSAMPLE:" + ratio_str(g_upsample_ratio) + "\r\nOK";
    }

    // ── AT+RATE? ──────────────────────────────────────────────
    else if (cu == "AT+RATE?") {
        uint32_t out = (g_upsample_ratio > 1)
                     ? g_sample_rate * g_upsample_ratio
                     : g_sample_rate;
        resp = "IN:"  + String(g_sample_rate) + " Hz\r\n"
             + "OUT:" + String(out)           + " Hz\r\nOK";
    }

    // ── AT+SOURCE? ────────────────────────────────────────────
    else if (cu == "AT+SOURCE?") {
        resp = "SOURCE:" + g_audio_source + "\r\nOK";
    }

    // ── AT+BTNAME= ────────────────────────────────────────────
    else if (cu.startsWith("AT+BTNAME=")) {
        String newName = cmd.substring(10);
        newName.trim();
        if (newName.length() == 0 || newName.length() > BLE_MAX_NAME_LEN) {
            resp = "ERROR: Name must be 1–" + String(BLE_MAX_NAME_LEN) + " chars";
        } else {
            g_bt_name = newName;
            g_prefs.putString(NVS_KEY_BT_NAME, g_bt_name);
            resp = "BT name set: " + g_bt_name + "\r\nRestart to apply.\r\nOK";
        }
    }
    else if (cu == "AT+BTNAME?") {
        resp = "BTNAME:" + g_bt_name + "\r\nOK";
    }

    // ── AT+RESTART ────────────────────────────────────────────
    else if (cu == "AT+RESTART") {
        debugPrint("Restarting in 1 s…");
        delay(1000);
        ESP.restart();
    }

    // ── AT+FACTORY ────────────────────────────────────────────
    else if (cu == "AT+FACTORY") {
        g_prefs.clear();
        debugPrint("Factory reset. Restarting…");
        delay(1000);
        ESP.restart();
    }

    // ── AT+BLEOFF / AT+BLEON ──────────────────────────────────
    else if (cu == "AT+BLEOFF") {
        BLEDevice::stopAdvertising();
        resp = "BLE advertising stopped\r\nOK";
    }
    else if (cu == "AT+BLEON") {
        BLEDevice::startAdvertising();
        resp = "BLE advertising started\r\nOK";
    }

    // ── Unknown ───────────────────────────────────────────────
    else {
        resp = "ERROR: Unknown command.\r\nType AT+HELP for list.";
    }

    debugPrint(resp);
}
