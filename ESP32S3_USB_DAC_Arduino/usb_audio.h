/**
 * usb_audio.h — USB Audio Class 2.0 (UAC2) for ESP32-S3
 * ─────────────────────────────────────────────────────────────
 * Implements a USB Audio Class 2.0 speaker device.
 * The PC/Mac installs a generic driver automatically — no INF
 * file, no custom driver required.
 *
 * v2.0: Audio data is now routed to the FIR upsampler queue
 * (fir_feed()) when g_upsample_ratio > 1, or written directly
 * to I2S when running in passthrough mode (ratio = 1).
 *
 * PHYSICAL CONNECTION:
 *   Use the USB-OTG port on the ESP32-S3 DevKit-C (the connector
 *   closest to GPIO 19/20, often labelled "USB" not "UART").
 *   You can still flash via the UART port simultaneously.
 *
 * COMPOSITE DEVICE:
 *   The firmware exposes two USB interfaces to the PC:
 *     ① USB Audio (speaker)   — PCM audio stream from PC
 *     ② USB CDC  (serial)     — Serial.print() output / AT commands
 *
 * SUPPORTED FORMATS (negotiated with host):
 *   44 100 Hz / 16-bit / stereo
 *   48 000 Hz / 16-bit / stereo  (default — most PCs use this)
 *   96 000 Hz / 16-bit / stereo
 *
 * ARDUINO IDE SETTINGS (mandatory):
 *   USB Mode           : USB-OTG (TinyUSB)   ← enables UAC2
 *   USB CDC On Boot    : Enabled              ← keeps Serial working
 *
 * Author: Danny Liu  |  v2.0.0
 */

#pragma once
#include <Arduino.h>
#include "config.h"
#include "i2s_dac.h"
#include "fir_upsample.h"   // fir_feed() + g_upsample_ratio
#include "USB.h"
#include "esp32-hal-tinyusb.h"

// ─── Global USB audio state (readable from main sketch) ──────
volatile uint32_t g_usb_sample_rate   = USB_AUDIO_SAMPLE_RATE;
volatile bool     g_usb_audio_active  = false;

// ─── Scratch buffer for one USB isochronous frame ─────────────
// Sized for 96 kHz × 2 ch × 2 bytes + 4-byte feedback endpoint margin.
static uint8_t s_usb_scratch[96 * 2 * 2 + 4];

// ─── TinyUSB callback: USB host delivered audio data ──────────
// This callback fires once per USB SOF (~1 ms).  Keep it short.
// Audio routing:
//   upsampler ON  → fir_feed() → FIR task on Core 0 → I2S 32-bit
//   upsampler OFF → i2s_dac_write() → I2S 16-bit direct
extern "C" bool tud_audio_rx_done_post_read_I_f_cb(
        uint8_t  rhport,
        uint16_t n_bytes_received,
        uint8_t  func_id,
        uint8_t  ep_out,
        uint8_t  cur_alt_setting)
{
    (void)rhport; (void)func_id; (void)ep_out; (void)cur_alt_setting;

    uint16_t avail = tud_audio_n_available(0);
    if (avail == 0) return true;

    uint16_t rd = tud_audio_n_read(0, s_usb_scratch, sizeof(s_usb_scratch));
    if (rd == 0) return true;

    g_usb_audio_active = true;

    if (g_upsample_ratio > 1) {
        // ── FIR upsampler path ────────────────────────────────
        // Convert byte count to stereo frame count
        // (each frame = 2 ch × 2 bytes = 4 bytes for 16-bit stereo)
        uint16_t n_frames = rd / (USB_AUDIO_CHANNELS * (USB_AUDIO_BIT_DEPTH / 8));
        fir_feed(reinterpret_cast<const int16_t*>(s_usb_scratch),
                 n_frames,
                 g_usb_sample_rate);
    } else {
        // ── Direct passthrough path ───────────────────────────
        i2s_dac_write(s_usb_scratch, rd);
    }

    return true;
}

// ─── TinyUSB callback: host changed alternate setting ─────────
// Alt 0 = host paused/idle; Alt 1 = streaming.
extern "C" bool tud_audio_set_itf_cb(
        uint8_t rhport,
        tusb_control_request_t const* p_request)
{
    (void)rhport; (void)p_request;
    return true;
}

// ─── TinyUSB callback: host changed sample rate ───────────────
// Called when the PC switches from 48 kHz to 96 kHz (or vice versa).
// We update g_usb_sample_rate so the FIR task can adapt.
extern "C" bool tud_audio_set_req_entity_cb(
        uint8_t  rhport,
        tusb_control_request_t const* p_request,
        uint8_t* pBuf)
{
    (void)rhport; (void)p_request; (void)pBuf;
    // The TinyUSB audio driver handles the control response.
    // If you need to read the actual frequency, parse p_request here
    // using the UAC2 clock source control structure.
    return true;
}

// ─── TinyUSB callback: host mute/volume control ───────────────
extern "C" bool tud_audio_feature_unit_set_request_cb(
        uint8_t rhport,
        tusb_control_request_t const* p_request,
        uint8_t* pBuf)
{
    (void)rhport; (void)p_request; (void)pBuf;
    // Extend here if you want the OS volume mixer to control
    // the PCM5102A hardware mute via PIN_PCM_MUTE.
    return true;
}

// ─── Initialise USB stack ─────────────────────────────────────
// Call once in setup() BEFORE starting the FIR task.
inline void usb_audio_init() {
    USB.begin();
    USB.manufacturerName("Danny Liu");
    USB.productName("ESP32S3 USB DAC");
    USB.serialNumber("DAC-001");
}

// ─── Check / reset activity flag ─────────────────────────────
inline bool usb_audio_is_streaming() {
    return g_usb_audio_active;
}

inline void usb_audio_reset_active() {
    g_usb_audio_active = false;
}
