/**
 * i2s_dac.h — I2S Driver for PCM5102A
 * ─────────────────────────────────────────────────────────────
 * Uses the ESP-IDF v5 I2S Standard (STD) API.
 * Compatible with ESP32 Arduino Core ≥ 2.0.11.
 *
 * v2.0: Added i2s_set_sample_rate_bits() for dynamic switching
 * between 16-bit USB passthrough mode and 32-bit upsampled mode
 * (required for 384 kHz FIR output).
 *
 * SUPPORTED OUTPUT CONFIGURATIONS:
 *   Passthrough : USB rate (48/96 kHz), 16-bit, stereo
 *   Upsampled 4×: 192 kHz, 32-bit, stereo
 *   Upsampled 8×: 384 kHz, 32-bit, stereo
 *
 * CLOCK SOURCE NOTES (384 kHz):
 *   BCK = 384 000 × 32 bit × 2 ch = 24.576 MHz  (within I2S limit)
 *   The I2S_CLK_SRC_APLL source is used when available so the
 *   audio PLL generates a bit-perfect 98.304 MHz MCLK.
 *   If APLL is unavailable or busy, the driver falls back to APB
 *   (80 MHz) with minor clock jitter — transparent via PCM5102A's
 *   internal PLL re-locking.
 *
 * HOW TO SWITCH BIT DEPTH:
 *   Call i2s_set_sample_rate_bits(rate, I2S_DATA_BIT_WIDTH_16BIT)
 *   for passthrough, or I2S_DATA_BIT_WIDTH_32BIT for upsampled.
 *   The function disables the channel, reconfigures, re-enables.
 *
 * Author: Danny Liu  |  v2.0.0
 */

#pragma once
#include <Arduino.h>
#include "config.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

// ─── Public I2S channel handle (used by FIR task and USB path) ─
extern i2s_chan_handle_t g_i2s_tx;
i2s_chan_handle_t g_i2s_tx = NULL;

// ─── Initialise I2S and PCM5102A mute GPIO ────────────────────
inline void i2s_dac_init() {

    // ── Mute GPIO ────────────────────────────────────────────
    // PCM5102A XSMT: HIGH = play, LOW = muted.
    // Remove if you tie XSMT permanently to 3.3 V.
    pinMode(PIN_PCM_MUTE, OUTPUT);
    digitalWrite(PIN_PCM_MUTE, HIGH);   // start unmuted

    // ── Create I2S TX channel ────────────────────────────────
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        I2S_NUM_0,       // Use I2S_NUM_1 if port 0 is taken.
        I2S_ROLE_MASTER  // ESP32-S3 generates BCK and WS.
    );
    chan_cfg.auto_clear        = true;           // zero-fill on underrun
    chan_cfg.dma_desc_num      = I2S_DMA_BUF_COUNT;
    chan_cfg.dma_frame_num     = I2S_DMA_BUF_LEN;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &g_i2s_tx, NULL));

    // ── Configure standard I2S (initial: USB passthrough rate) ─
    i2s_std_config_t std_cfg = {
        // Clock: start at USB rate; i2s_set_sample_rate_bits()
        // will switch to 384 kHz once the FIR task starts.
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(USB_AUDIO_SAMPLE_RATE),

        // Slot: 16-bit stereo initially; FIR path switches to 32-bit.
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO
        ),

        .gpio_cfg = {
#ifdef PIN_I2S_MCLK
            .mclk  = (gpio_num_t)PIN_I2S_MCLK,
#else
            .mclk  = I2S_GPIO_UNUSED,   // PCM5102A uses internal PLL
#endif
            .bclk  = (gpio_num_t)PIN_I2S_BCK,    // ← config.h PIN_I2S_BCK
            .ws    = (gpio_num_t)PIN_I2S_WS,      // ← config.h PIN_I2S_WS
            .dout  = (gpio_num_t)PIN_I2S_DOUT,    // ← config.h PIN_I2S_DOUT
            .din   = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,  // set true if DAC expects inverted LRCK
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(g_i2s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(g_i2s_tx));
}

// ─── Runtime reconfiguration: sample rate AND bit depth ───────
// Call whenever upsampling ratio changes.
//   rate  : target output rate (e.g. 384000, 192000, 48000)
//   bits  : I2S_DATA_BIT_WIDTH_16BIT  or  I2S_DATA_BIT_WIDTH_32BIT
//
// The channel is disabled during reconfiguration; I2S output goes
// silent for ~1 ms (inaudible as a soft click when muted first).
inline void i2s_set_sample_rate_bits(uint32_t rate,
                                     i2s_data_bit_width_t bits) {
    i2s_channel_disable(g_i2s_tx);

    // ── Clock reconfiguration ────────────────────────────────
    // For 384 kHz, request APLL source for bit-perfect clocking.
    // If your ESP-IDF version does not support APLL on S3,
    // comment out the if-block and use the default config only.
    i2s_std_clk_config_t clk;
#if SOC_I2S_SUPPORTS_APLL
    if (rate >= 192000) {
        // APLL can synthesise exact audio multiples (98.304 MHz for 384 kHz)
        clk = I2S_STD_CLK_DEFAULT_CONFIG(rate);
        clk.clk_src = I2S_CLK_SRC_APLL;
    } else {
        clk = I2S_STD_CLK_DEFAULT_CONFIG(rate);
    }
#else
    clk = I2S_STD_CLK_DEFAULT_CONFIG(rate);
    // Note: APB (80 MHz) at 384 kHz has ~0.01 % jitter.
    // PCM5102A's internal PLL absorbs this transparently.
#endif
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(g_i2s_tx, &clk));

    // ── Slot reconfiguration ─────────────────────────────────
    // 32-bit slots used for upsampled output so DMA word-alignment
    // is maintained and PCM5102A receives full-range samples.
    i2s_std_slot_config_t slot =
        I2S_STD_MSB_SLOT_DEFAULT_CONFIG(bits, I2S_SLOT_MODE_STEREO);
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(g_i2s_tx, &slot));

    i2s_channel_enable(g_i2s_tx);

    Serial.printf("[I2S]  Reconfigured: %u Hz, %d-bit\n", rate,
                  (bits == I2S_DATA_BIT_WIDTH_32BIT) ? 32 : 16);
}

// ─── Write 16-bit PCM (USB passthrough path) ──────────────────
inline size_t i2s_dac_write(const void* buf, size_t bytes) {
    size_t written = 0;
    i2s_channel_write(g_i2s_tx, buf, bytes, &written, pdMS_TO_TICKS(20));
    return written;
}

// ─── Write 32-bit PCM (FIR upsampler path) ────────────────────
// buf  : pointer to interleaved int32_t stereo samples.
// bytes: byte count = n_stereo_frames × 2 × sizeof(int32_t).
inline size_t i2s_dac_write_i32(const int32_t* buf, size_t bytes) {
    size_t written = 0;
    // 50 ms timeout: gives enough slack at 384 kHz with 16 DMA buffers.
    // If you see timeout warnings, increase I2S_DMA_BUF_COUNT in config.h.
    i2s_channel_write(g_i2s_tx, buf, bytes, &written, pdMS_TO_TICKS(50));
    return written;
}

// ─── Runtime sample-rate change only (no bit-depth change) ────
// Legacy helper kept for passthrough rate updates.
inline void i2s_set_sample_rate(uint32_t rate) {
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(rate);
    i2s_channel_disable(g_i2s_tx);
    i2s_channel_reconfig_std_clock(g_i2s_tx, &clk);
    i2s_channel_enable(g_i2s_tx);
}

// ─── Hardware mute / unmute via PCM5102A XSMT ─────────────────
inline void i2s_dac_set_mute(bool mute) {
    digitalWrite(PIN_PCM_MUTE, mute ? LOW : HIGH);
}
