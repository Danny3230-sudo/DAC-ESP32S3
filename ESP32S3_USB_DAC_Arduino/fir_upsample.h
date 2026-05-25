/**
 * fir_upsample.h — Polyphase FIR Upsampler (Core 0)
 * ─────────────────────────────────────────────────────────────
 * Implements a Kaiser-windowed polyphase FIR interpolator that
 * upsamples 16-bit PCM audio from the USB rate (48 / 96 kHz)
 * to a higher output rate (192 / 384 kHz) for the PCM5102A DAC.
 *
 * ARCHITECTURE:
 * ┌──────────────┐  queue  ┌────────────────────────────────────┐
 * │ Core 1 (USB) │ ──────► │ Core 0 — fir_task()                │
 * │ usb_audio.h  │         │  polyphase FIR → i2s_dac_write_i32 │
 * └──────────────┘         └────────────────────────────────────┘
 *
 * WHY POLYPHASE?
 *   A naïve upsample-then-filter approach would run the FIR at
 *   the high output rate (384 kHz), wasting L-1 out of every L
 *   multiply-accumulate operations on zero-valued inserted samples.
 *   The polyphase decomposition runs K MACs per output sample
 *   regardless of L, giving the same result with L× less work.
 *
 * FILTER DESIGN:
 *   Type    : Linear-phase FIR low-pass
 *   Window  : Kaiser  (β = FIR_KAISER_BETA, default 7.0 → ~80 dB)
 *   Cutoff  : Nyquist of input signal (½ × USB sample rate)
 *   Taps    : FIR_MAX_L × FIR_MAX_K  = 128  (default)
 *   Phases  : FIR_MAX_L  = 8  (one per upsampled output slot)
 *   K/phase : FIR_MAX_K  = 16
 *
 *   Increasing FIR_MAX_K in config.h to 24 raises the stop-band
 *   to ~100 dB with about 50 % more CPU on Core 0.
 *
 * CPU BUDGET (Core 0 @ 240 MHz, 8× upsample, 48 kHz input):
 *   Per input sample : L × K × 2 ch = 8 × 16 × 2 = 256 MACs
 *   Throughput needed: 48 000 × 256 = 12.3 M MACs/s
 *   ESP32-S3 FPU cap : ~100 M MACs/s (float32, single-issue)
 *   Headroom         : ~88 % CPU free on Core 0 after upsampling
 *
 * UPSAMPLING MODES (changed via AT+UPSAMPLE or at boot):
 *   OFF (1×) → 48 kHz passthrough  — 16-bit direct to I2S
 *   4×       → 48 kHz → 192 kHz   — 32-bit to I2S
 *   8×       → 48 kHz → 384 kHz   — 32-bit to I2S   ← default
 *   (96 kHz USB input × 4× = 384 kHz also works)
 *
 * Author: Danny Liu  |  v2.0.0
 */

#pragma once
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "config.h"
#include "i2s_dac.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ══════════════════════════════════════════════════════════════
// PUBLIC TYPES & HANDLES
// ══════════════════════════════════════════════════════════════

/** Inter-core audio packet.
 *  One USB audio frame (≈ 1 ms of audio) per item. */
typedef struct {
    int16_t  buf[FIR_QUEUE_ITEM_FRAMES * 2]; // interleaved L/R, 16-bit
    uint16_t n_frames;                        // number of stereo frames
    uint32_t src_rate;                        // USB source rate (Hz)
} fir_item_t;

/** Public queue handle — written by Core 1 (USB callback),
 *  read by Core 0 (FIR task). */
QueueHandle_t    g_fir_queue     = nullptr;

/** Current upsampling ratio. 1 = passthrough, 4 = 4×, 8 = 8×.
 *  Write from anywhere; the FIR task detects changes each packet. */
volatile uint8_t g_upsample_ratio = FIR_DEFAULT_RATIO;

// ══════════════════════════════════════════════════════════════
// INTERNAL DSP STATE  (all in Core 0 context — no locking needed)
// ══════════════════════════════════════════════════════════════

/** Full linear coefficient array h[0 .. L*K-1].
 *  Generated at boot and whenever the ratio changes. */
static float s_h[FIR_MAX_L * FIR_MAX_K];

/** Polyphase matrix. s_poly[p][k] = h[p + k*L].
 *  Phase p is applied to produce the p-th upsampled output
 *  for each input sample. */
static float s_poly[FIR_MAX_L][FIR_MAX_K];

/** Per-channel circular delay line (history of input samples). */
typedef struct {
    float delay[FIR_MAX_K]; // oldest … newest
    int   wr;               // write pointer (index of next slot to fill)
} poly_ch_t;

static poly_ch_t s_ch_L; // left channel state
static poly_ch_t s_ch_R; // right channel state

/** Ratio that was last used to build the polyphase matrix. */
static uint8_t   s_built_ratio = 0;

/** Output buffer: max stereo frames = FIR_QUEUE_ITEM_FRAMES × FIR_MAX_L.
 *  Declared static so it lives in DRAM, not on the task stack. */
static int32_t   s_out[FIR_QUEUE_ITEM_FRAMES * FIR_MAX_L * 2];

// ══════════════════════════════════════════════════════════════
// FILTER MATH
// ══════════════════════════════════════════════════════════════

/** Modified Bessel function I₀(x) via power series.
 *  Used for Kaiser window computation.
 *  Converges to float precision in ≤ 20 iterations for β ≤ 10. */
static float bessel_i0(float x) {
    float result = 1.0f;
    float term   = 1.0f;
    float hx     = x * 0.5f;
    for (int k = 1; k <= 25; k++) {
        float c = hx / (float)k;
        term  *= c * c;
        result += term;
        if (term < 1e-10f) break;   // converged
    }
    return result;
}

/** Generate Kaiser-windowed sinc coefficients into h[0..L*K-1].
 *
 *  Each coefficient:
 *    h[n] = (1/L) × sinc((n − N/2) / L) × w_kaiser(n, N, β)
 *
 *  The 1/L normalisation ensures that a DC signal passes through
 *  with unity gain after upsampling (all L phases sum to 1.0).
 *
 *  @param h      Output array, length L×K.
 *  @param L      Upsampling factor.
 *  @param K      Polyphase taps per branch.
 *  @param beta   Kaiser β parameter.
 */
static void fir_generate(float* h, int L, int K, float beta) {
    int   N      = L * K;
    float center = (N - 1) * 0.5f;
    float i0b    = bessel_i0(beta);

    for (int n = 0; n < N; n++) {
        // ── Sinc term ────────────────────────────────────────
        float w   = (float)n - center;          // offset from centre
        float arg = (float)M_PI * w / (float)L; // normalised to output rate
        float sinc = (fabsf(w) < 1e-6f)
                   ? (1.0f / (float)L)
                   : (sinf(arg) / (arg * (float)L));

        // ── Kaiser window ────────────────────────────────────
        float norm   = (n - center) / center;   // ∈ [-1, +1]
        float arg_w  = beta * sqrtf(fmaxf(0.0f, 1.0f - norm * norm));
        float kaiser = bessel_i0(arg_w) / i0b;

        h[n] = sinc * kaiser;
    }
}

/** Decompose linear h[] into the polyphase matrix.
 *  s_poly[p][k] = h[p + k*L]  for p ∈ [0,L), k ∈ [0,K). */
static void fir_build_polyphase(int L, int K) {
    for (int p = 0; p < L; p++)
        for (int k = 0; k < K; k++)
            s_poly[p][k] = s_h[p + k * L];
}

/** Full rebuild: regenerate coefficients + polyphase matrix + reset
 *  delay lines.  Called at boot and whenever ratio changes.
 *
 *  HOW TO TUNE:
 *    • To widen the pass-band (allow more HF into the stop-band),
 *      reduce FIR_KAISER_BETA.
 *    • To increase stop-band rejection, raise both FIR_KAISER_BETA
 *      and FIR_MAX_K (more taps = sharper filter, more CPU).
 */
static void fir_rebuild(uint8_t L) {
    int K = FIR_MAX_K;

    Serial.printf("[FIR]  Rebuilding polyphase matrix: L=%d K=%d β=%.1f\n",
                  L, K, FIR_KAISER_BETA);

    fir_generate(s_h, L, K, FIR_KAISER_BETA);
    fir_build_polyphase(L, K);

    // Reset both delay lines (silence history)
    memset(&s_ch_L, 0, sizeof(poly_ch_t));
    memset(&s_ch_R, 0, sizeof(poly_ch_t));

    s_built_ratio = L;

    uint32_t in_rate  = USB_AUDIO_SAMPLE_RATE;
    uint32_t out_rate = in_rate * L;
    Serial.printf("[FIR]  Filter ready: %u Hz → %u Hz  (×%d)\n",
                  in_rate, out_rate, L);
}

// ══════════════════════════════════════════════════════════════
// POLYPHASE PROCESSING (inner loop — called per input sample)
// ══════════════════════════════════════════════════════════════

/**
 * Process one input sample through all L polyphase branches.
 *
 * For each input sample x[n] the function:
 *   1. Inserts x[n] into the circular delay line.
 *   2. Computes L output samples  y[n*L+p]  (p = 0 … L-1),
 *      each using a different polyphase subfilter row.
 *
 * Polyphase output formula:
 *   y[n*L + p] = Σ_{k=0}^{K-1}  s_poly[p][k] × x[n-k]
 *
 * @param ch       Pointer to the channel delay-line state.
 * @param L        Current upsampling factor.
 * @param in       Input sample value (float, ±1.0 normalised).
 * @param out      Output array [L]; caller provides storage.
 */
static IRAM_ATTR void poly_process(poly_ch_t* ch, int L,
                                   float in, float* out) {
    // Insert new sample at write pointer
    ch->delay[ch->wr] = in;
    int wr = ch->wr;
    ch->wr = (ch->wr + 1) % FIR_MAX_K;

    // One MAC loop per polyphase branch
    for (int p = 0; p < L; p++) {
        float acc = 0.0f;
        const float* coeff = s_poly[p];
        // Walk delay line from oldest sample (wr+1) to newest (wr)
        for (int k = 0; k < FIR_MAX_K; k++) {
            int idx = (wr + 1 + k) % FIR_MAX_K;
            acc += coeff[k] * ch->delay[idx];
        }
        out[p] = acc;
    }
}

// ══════════════════════════════════════════════════════════════
// FREERTOS TASK  (Core 0)
// ══════════════════════════════════════════════════════════════

/** FIR upsampling task.
 *  Blocks on the queue, processes each USB audio packet, and
 *  writes the upsampled 32-bit PCM to the I2S DMA. */
static void fir_task(void* /*arg*/) {
    fir_item_t item;
    float      tmp_L[FIR_MAX_L]; // per-sample polyphase outputs (left)
    float      tmp_R[FIR_MAX_L]; // per-sample polyphase outputs (right)

    // Initial matrix build for the boot ratio
    fir_rebuild(g_upsample_ratio);

    // Bring I2S up to the upsampled rate immediately
    if (g_upsample_ratio > 1) {
        uint32_t out_rate = USB_AUDIO_SAMPLE_RATE * g_upsample_ratio;
        i2s_set_sample_rate_bits(out_rate, I2S_DATA_BIT_WIDTH_32BIT);
    }

    for (;;) {
        // Block here until Core 1 deposits a USB packet
        if (xQueueReceive(g_fir_queue, &item, portMAX_DELAY) != pdTRUE)
            continue;

        uint8_t L = g_upsample_ratio;

        // ── Ratio change detected? ───────────────────────────
        if (L != s_built_ratio) {
            fir_rebuild(L);
            uint32_t out_rate = (L > 1) ? (item.src_rate * L) : item.src_rate;
            i2s_data_bit_width_t bits = (L > 1)
                ? I2S_DATA_BIT_WIDTH_32BIT
                : I2S_DATA_BIT_WIDTH_16BIT;
            i2s_set_sample_rate_bits(out_rate, bits);
        }

        // ── Passthrough mode (L = 1): direct 16-bit write ────
        if (L == 1) {
            i2s_dac_write(item.buf,
                          item.n_frames * 2 * sizeof(int16_t));
            continue;
        }

        // ── Upsampling mode: polyphase FIR ───────────────────
        // Output buffer index; one int32_t per channel per output frame.
        int out_idx = 0;

        for (int f = 0; f < item.n_frames; f++) {
            // Normalise 16-bit input to float ±1.0
            float in_L = (float)item.buf[f * 2    ] * (1.0f / 32768.0f);
            float in_R = (float)item.buf[f * 2 + 1] * (1.0f / 32768.0f);

            // Run both channels through the polyphase filter
            poly_process(&s_ch_L, L, in_L, tmp_L);
            poly_process(&s_ch_R, L, in_R, tmp_R);

            // Pack L output frames into the 32-bit output buffer
            for (int p = 0; p < L; p++) {
                // Clamp and scale float → int32 full range
                float sl = fmaxf(-1.0f, fminf(1.0f, tmp_L[p]));
                float sr = fmaxf(-1.0f, fminf(1.0f, tmp_R[p]));
                s_out[out_idx++] = (int32_t)(sl * 2147483647.0f);
                s_out[out_idx++] = (int32_t)(sr * 2147483647.0f);
            }
        }

        // Write the entire upsampled packet to I2S DMA in one call
        i2s_dac_write_i32(s_out,
                          (size_t)out_idx * sizeof(int32_t));
    }
}

// ══════════════════════════════════════════════════════════════
// PUBLIC API
// ══════════════════════════════════════════════════════════════

/** Initialise the FIR upsampler.
 *  Creates the inter-core queue and spawns the FIR task on Core 0.
 *  Call once from setup() AFTER i2s_dac_init().
 *
 *  @param ratio  Initial upsampling ratio: 1 (off), 4, or 8.
 */
inline void fir_upsample_init(uint8_t ratio) {
    g_upsample_ratio = ratio;

    g_fir_queue = xQueueCreate(FIR_QUEUE_DEPTH, sizeof(fir_item_t));
    if (!g_fir_queue) {
        Serial.println("[FIR]  ERROR: queue creation failed!");
        return;
    }

    // Pin to Core 0, priority 5 (above loop task priority of 1).
    // Stack 8192 bytes: enough for delay lines, temp arrays, locals.
    // To adjust priority:  raise if you get I2S underruns under load,
    //                      lower if Core 0 network tasks starve.
    BaseType_t ok = xTaskCreatePinnedToCore(
        fir_task,        // task function
        "fir_upsample",  // name (for debugging)
        8192,            // stack depth (bytes on ESP32)
        nullptr,         // no arg needed
        5,               // priority
        nullptr,         // task handle (not stored)
        0                // Core 0  ← USB audio stays on Core 1
    );

    if (ok != pdPASS)
        Serial.println("[FIR]  ERROR: task creation failed!");
    else
        Serial.println("[FIR]  Task started on Core 0.");
}

/** Feed one USB audio packet into the upsampler queue.
 *  Called from the USB audio callback on Core 1.
 *
 *  @param pcm_stereo  Interleaved 16-bit L/R samples.
 *  @param n_frames    Number of stereo frames in pcm_stereo.
 *  @param src_rate    Source sample rate (Hz), e.g. 48000.
 *  @return true if queued, false if queue was full (packet dropped).
 *
 *  Dropping is preferable to blocking the USB callback, which runs
 *  in an ISR context.  Queue full means Core 0 is not keeping up —
 *  reduce FIR_MAX_K or raise FIR task priority in config.h.
 */
inline bool fir_feed(const int16_t* pcm_stereo,
                     uint16_t n_frames,
                     uint32_t src_rate) {
    if (!g_fir_queue) return false;

    fir_item_t item;
    item.n_frames = (n_frames <= FIR_QUEUE_ITEM_FRAMES)
                  ? n_frames : FIR_QUEUE_ITEM_FRAMES;
    item.src_rate = src_rate;
    memcpy(item.buf, pcm_stereo,
           item.n_frames * 2 * sizeof(int16_t));

    // fromISR variant: safe to call from USB callback context.
    // portYIELD_FROM_ISR handled internally by FreeRTOS.
    return xQueueSendToBackFromISR(g_fir_queue, &item, nullptr)
           == pdTRUE;
}

/** Change the upsampling ratio at runtime.
 *  The FIR task detects the change on its next iteration and
 *  rebuilds the polyphase matrix + reconfigures I2S automatically.
 *
 *  @param ratio  1 (passthrough), 4 (×4), or 8 (×8).
 */
inline void fir_set_ratio(uint8_t ratio) {
    if (ratio != 1 && ratio != 4 && ratio != 8) {
        Serial.println("[FIR]  Invalid ratio. Use 1, 4, or 8.");
        return;
    }
    g_upsample_ratio = ratio;
    Serial.printf("[FIR]  Ratio change queued: ×%d\n", ratio);
}
