/**
 * ╔══════════════════════════════════════════════════════════════╗
 * ║  config.h — Hardware Pin & System Configuration              ║
 * ║  ESP32-S3 USB Audio DAC + PCM5102A + FIR Upsampler          ║
 * ║  Author: Danny Liu  |  v2.0.0                               ║
 * ╚══════════════════════════════════════════════════════════════╝
 *
 * HOW TO CHANGE GPIO PINS
 * ─────────────────────────────────────────────────────────────────
 * All hardware pin assignments are centralised here.
 * Change the number, recompile, flash. No other file needs editing.
 *
 * ESP32-S3 GPIO restrictions:
 *   GPIO  0        Strapping (boot mode). AVOID.
 *   GPIO  3        Strapping. AVOID.
 *   GPIO 19 / 20   USB D-/D+ (USB-OTG). DO NOT reassign.
 *   GPIO 45 / 46   Strapping. AVOID.
 *   GPIO 26–32     May be used by PSRAM on some modules. Verify.
 *   GPIO 33–37     May be used by Octal SPI flash. Verify.
 *   Safe GPIOs     1–18, 21, 35–44, 47–48 (check your specific board).
 *
 * PCM5102A wiring quick-reference
 * ─────────────────────────────────────────────────────────────────
 *   PCM5102A Pin   ESP32-S3 GPIO   Notes
 *   ─────────────────────────────────────────────────────────────
 *   VCC            3.3 V           3.3 V preferred (5 V also works)
 *   GND            GND
 *   BCK            GPIO 14         I2S Bit Clock  ← PIN_I2S_BCK
 *   LRCK           GPIO 11         Word Select    ← PIN_I2S_WS
 *   DIN            GPIO 13         Serial Data    ← PIN_I2S_DOUT
 *   SCK (SCKI)     NC or GND       Internal PLL; do not drive unless
 *                                  MCLK output is uncommented below
 *   FMT            GND             I2S format (LOW = standard I2S)
 *   DEMP           GND             De-emphasis off
 *   XSMT           GPIO 10         Soft-mute (HIGH=play, LOW=mute)
 *                                  ← PIN_PCM_MUTE; tie HIGH if unused
 */

#pragma once

// ═══════════════════════════════════════════════════════════════
// I2S / PCM5102A GPIO ASSIGNMENTS
// ═══════════════════════════════════════════════════════════════

/** Bit Clock → PCM5102A BCK.
 *  Change: replace 14 with any free GPIO. */
#define PIN_I2S_BCK      14

/** Word Select → PCM5102A LRCK.
 *  Change: replace 11 with any free GPIO. */
#define PIN_I2S_WS       11

/** Data Out → PCM5102A DIN.
 *  Change: replace 13 with any free GPIO. */
#define PIN_I2S_DOUT     13

/** Master Clock (optional — PCM5102A does NOT need it).
 *  Uncomment only if your DAC module requires an external MCLK.
 *  Typical ratio: MCLK = 256 × sample_rate.
 *  Also update i2s_gpio_cfg_t.mclk in i2s_dac.h. */
// #define PIN_I2S_MCLK  14

/** Soft-mute control → PCM5102A XSMT.
 *  HIGH = play, LOW = mute. Change: replace 10 with any free GPIO.
 *  To remove software mute: tie XSMT pin to 3.3 V permanently and
 *  delete all references to PIN_PCM_MUTE. */
#define PIN_PCM_MUTE     10

// ═══════════════════════════════════════════════════════════════
// USB AUDIO PARAMETERS (received from PC)
// ═══════════════════════════════════════════════════════════════

/** Sample rate the device advertises to the USB host.
 *  Most PCs default to 48 000 Hz.
 *  Valid values for UAC2: 44100, 48000, 96000.
 *  This is the INPUT rate; the upsampler converts it to a higher rate. */
#define USB_AUDIO_SAMPLE_RATE   48000

/** Channels (2 = stereo). */
#define USB_AUDIO_CHANNELS      2

/** USB bit depth (16-bit is universal; PCM5102A supports up to 32). */
#define USB_AUDIO_BIT_DEPTH     16

/** I2S DMA buffer length in samples (per channel).
 *  Increase if you hear underruns at 384 kHz output.
 *  Must be a power of 2. */
#define I2S_DMA_BUF_LEN        512

/** Number of DMA buffers. Total latency ≈ DMA_LEN × DMA_COUNT / rate. */
#define I2S_DMA_BUF_COUNT        8

// ═══════════════════════════════════════════════════════════════
// POLYPHASE FIR UPSAMPLER PARAMETERS
// ═══════════════════════════════════════════════════════════════

/**
 * Maximum upsampling ratio supported.
 * At 8× and 48 kHz USB input → 384 kHz I2S output.
 * Governs compile-time array sizes — do not reduce below your
 * intended maximum ratio.
 */
#define FIR_MAX_L               8

/**
 * Polyphase taps per branch (K).
 * Total FIR taps = FIR_MAX_L × FIR_MAX_K = 128.
 * Increasing K improves stop-band rejection at the cost of CPU.
 * K=16 → ~80 dB stopband; K=24 → ~100 dB; K=32 → ~120 dB.
 * At 240 MHz, K=16 leaves ~70 % CPU headroom on Core 0.
 */
#define FIR_MAX_K               16

/**
 * Kaiser window shape parameter (β).
 * Higher β = more stop-band attenuation, wider transition band.
 *   β = 5.0  →  ~57 dB   (fast transition)
 *   β = 7.0  →  ~80 dB   ← default: excellent HiFi quality
 *   β = 9.0  →  ~100 dB  (use with FIR_MAX_K ≥ 24)
 * Change here only; no other file needs updating.
 */
#define FIR_KAISER_BETA         7.0f

/**
 * Default upsampling ratio at boot.
 * 8 → 48 kHz USB input becomes 384 kHz I2S output.
 * 4 → 48 kHz → 192 kHz, or 96 kHz → 384 kHz.
 * 1 → passthrough (no upsampling, direct 16-bit to I2S).
 * Override at runtime: AT+UPSAMPLE=8X / 4X / OFF
 */
#define FIR_DEFAULT_RATIO        8

/**
 * FreeRTOS queue depth between Core 1 (USB) and Core 0 (FIR).
 * Each slot holds one USB audio frame (~1 ms of audio).
 * 16 slots = 16 ms of buffering — enough to absorb USB timing jitter.
 * Reduce to 8 if you want lower latency and your host delivers
 * consistently-sized packets.
 */
#define FIR_QUEUE_DEPTH         16

/**
 * Maximum stereo frames per queue item.
 * 96 covers one 1 ms USB frame at 96 kHz (the highest USB rate).
 * At 48 kHz a frame contains 48 stereo samples — well within this limit.
 * Do not reduce below 96 if you intend to use 96 kHz USB input.
 */
#define FIR_QUEUE_ITEM_FRAMES   96

// ═══════════════════════════════════════════════════════════════
// BLE CONFIGURATION
// ═══════════════════════════════════════════════════════════════

/** BLE device name shown to Android during scan.
 *  Override at runtime: AT+BTNAME=<name>  (persists across reboots). */
#define BLE_DEFAULT_NAME        "ESP32S3-DAC"

/** Maximum BLE name length in characters. */
#define BLE_MAX_NAME_LEN         20

/** Nordic UART Service (NUS) UUIDs — industry-standard, used by most
 *  BLE serial apps and the PWA's Web Bluetooth implementation. */
#define BLE_NUS_SERVICE_UUID    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_NUS_TX_UUID         "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 → App
#define BLE_NUS_RX_UUID         "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // App → ESP32

/** Interval between automatic BLE status-push packets (milliseconds). */
#define BLE_STATUS_INTERVAL_MS  3000

// ═══════════════════════════════════════════════════════════════
// SERIAL / DEBUG
// ═══════════════════════════════════════════════════════════════

/** UART baud rate for PC serial terminal.
 *  With "USB CDC On Boot: Enabled", maps to the native USB CDC port. */
#define SERIAL_BAUD_RATE        115200

// ═══════════════════════════════════════════════════════════════
// FIRMWARE IDENTITY
// ═══════════════════════════════════════════════════════════════

#define FIRMWARE_VERSION        "2.0.0"
#define DEVICE_MODEL            "ESP32S3-USB-DAC"
#define DEVICE_MANUF            "Danny Liu"

// ═══════════════════════════════════════════════════════════════
// NVS KEYS (Preferences library storage namespace & keys)
// ═══════════════════════════════════════════════════════════════

#define NVS_NAMESPACE           "dac_cfg"
#define NVS_KEY_BT_NAME         "btname"
#define NVS_KEY_MUTE            "mute"
#define NVS_KEY_UPSAMPLE        "upsample"
