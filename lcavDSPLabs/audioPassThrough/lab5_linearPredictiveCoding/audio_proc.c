#include "arm_math_types.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"

#include "arm_math.h"

#include "audio_bus.h"
#include "audio_usb.h"
#include "ring_buffer.h"
#include <stdint.h>

/* Constants and Macros */
#define BLOCK_SIZE AUDIO_PACKET_SAMPLES
#define GRAIN_LEN_MS 20
#define STRIDE_MS 17
#define PITCH_SHIFT_FACTOR 0.6f
#define GRAIN_LEN_SAMPLES (AUDIO_SAMPLES_PER_MS * GRAIN_LEN_MS)
#define STRIDE_SAMPLES (AUDIO_SAMPLES_PER_MS * STRIDE_MS)
#define OVERLAP_LEN (GRAIN_LEN_SAMPLES - STRIDE_SAMPLES - 1)
#define LPC_ORDER 32
#define IIR_LPC_STAGES (LPC_ORDER / 2)
#define FIR_MULTICORE_SPLIT_INDEX (GRAIN_LEN_SAMPLES / 2)
#define FIR_CORE0_NUM_TAPS (LPC_ORDER / 2)
#define FIR_CORE1_NUM_TAPS ((LPC_ORDER / 2) + 1)

/* State Variables */
// https://arm-software.github.io/CMSIS-DSP/latest/group__BiquadCascadeDF2T.html
static float32_t iir_dc_block_coeffs[5] = {
    1, -1, 0,  // b0, b1, b2
    0.9993f, 0 // a1, a2
};
static float32_t iir_dc_block_state[2];
static arm_biquad_cascade_df2T_instance_f32 iir_dc_block_instance = {
    .numStages = 1, .pCoeffs = iir_dc_block_coeffs, .pState = iir_dc_block_state};

static float32_t taper_window[GRAIN_LEN_SAMPLES];                       // 4KB
static uint16_t interpolation_indices[GRAIN_LEN_SAMPLES];               // 2KB
static float32_t interpolation_amps[GRAIN_LEN_SAMPLES];                 // 4KB
static float32_t grain_overlap[OVERLAP_LEN] = {0};                      // 0.4KB
rb_init_static_size_aligned(x_concat, (1 << 11), sizeof(float32_t));    // 8KB Supports grains of up to 42ms
rb_init_static_size_aligned(output_fifo, (1 << 11), sizeof(float32_t)); // 8KB
static float32_t input_grain_buffer[GRAIN_LEN_SAMPLES];                 // 8KB
static float32_t lpc_excitation[GRAIN_LEN_SAMPLES];                     // 8KB
static float32_t fir_lpc_analysis_state_core0[GRAIN_LEN_SAMPLES / 2 + LPC_ORDER];
static float32_t fir_lpc_analysis_state_core1[GRAIN_LEN_SAMPLES / 2 + LPC_ORDER];
static float32_t fir_lpc_analysis_coeffs[LPC_ORDER];
static arm_fir_instance_f32 fir_lpc_analysis_instance_core0;
static arm_fir_instance_f32 fir_lpc_analysis_instance_core1;
static float32_t iir_lpc_synthesis_state[GRAIN_LEN_SAMPLES + LPC_ORDER];
static float32_t fir_global_history[FIR_CORE0_NUM_TAPS];
static float32_t autocorrelation_vector[LPC_ORDER + 1];

static uint32_t dma_write_rb_chan, dma_read_rb_chan;
static dma_channel_config dma_write_rb_config, dma_read_rb_config;

static queue_t autocorrelation_queue;
static queue_t fir_lpc_queue;
static queue_t results_queue;

/* Structs */
typedef struct {
  uint32_t start_delay;
  uint32_t end_delay;
} autocorr_job_t;

static autocorr_job_t autocorr_job;

/* Private Helper Functions */
// Lightweight RNG for dithering
static inline uint32_t fast_rand(uint32_t *state) {
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

// Generate uniform float in range [0.0, 1.0)
static inline float rand_uniform(uint32_t *state) {
  // 24 bits are extracted because that is the precision if an f32 mantissa
  return (fast_rand(state) & 0xFFFFFF) / 16777216.0f; // 2^24
}

// Convert float in range [-1.0, 1.0] to PCM16 with TPDF dithering
static inline int16_t float_to_pcm16_dither(float x, uint32_t *rng_state) {
  // TPDF dithering
  float dither = (rand_uniform(rng_state) - rand_uniform(rng_state)) * (1.0f / 32768.0f);

  x += dither;

  if (x >= 1.0f)
    x = 1.0f;
  if (x < -1.0f)
    x = -1.0f;

  // round to the nearest integer using 1 hardware instruction
  int32_t result;
  float32_t scaled_x = x * 32767.0f; // we scale back to full range PCM16 here
  asm("vcvtn.s32.f32 %1, %1\n"
      "vmov %0, %1\n"
      : "=r"(result), "+w"(scaled_x)
      :);
  return result;

  return (int16_t)result;
}

static inline void write_to_ring_buffer_with_dma(ring_buffer_t *rb, float32_t *data, uint16_t length) {
  assert(size(rb) + length <= rb->capacity);
  dma_channel_configure(dma_write_rb_chan, &dma_write_rb_config, rb_get_write_buffer(rb), data, length, true);
  dma_channel_wait_for_finish_blocking(dma_write_rb_chan);
  rb_increase_write_index(rb, length);
}

static inline void write_to_grain_buffer_from_dma(ring_buffer_t *rb, float32_t *data, uint16_t length) {
  assert(size(rb) >= length);
  dma_channel_configure(dma_read_rb_chan, &dma_read_rb_config, data, rb_get_read_buffer(rb), length, true);
  dma_channel_wait_for_finish_blocking(dma_read_rb_chan);
}

// The reason the CMSIS-DSP function is not used is that it always the correlation with maxDelay = 2 * len - 1 which for
// a grain length is too much to calculate in 1 ms.
static inline void autocorrelate(const float32_t *src, float32_t src_len, float32_t *dst, uint32_t start_delay,
                                 uint32_t end_delay) {
  float32_t inv_len = 1.0f / (float32_t)src_len;

  for (uint32_t delay = start_delay; delay < end_delay; delay++) {
    uint32_t overlap_len = src_len - delay; // Sliding windows length
    const float32_t *x_ptr = src;
    const float32_t *x_ptr_delayed = src + delay;

    float32_t acc;
    arm_dot_prod_f32(x_ptr, x_ptr_delayed, overlap_len, &acc);
    dst[delay] = acc * inv_len;
  }
}

void launch_core1() {
  while (1) {
    if (!queue_is_empty(&autocorrelation_queue)) {
      autocorr_job_t job;
      queue_remove_blocking(&autocorrelation_queue, &job);
      autocorrelate(input_grain_buffer, GRAIN_LEN_SAMPLES, autocorrelation_vector, job.start_delay, job.end_delay);
      int32_t result = 0;
      queue_add_blocking(&results_queue, &result);
    } else if (!queue_is_empty(&fir_lpc_queue)) {
      uint32_t trigger;
      queue_remove_blocking(&fir_lpc_queue, &trigger);
      arm_fir_f32(&fir_lpc_analysis_instance_core1, &input_grain_buffer[FIR_MULTICORE_SPLIT_INDEX],
                  &lpc_excitation[FIR_MULTICORE_SPLIT_INDEX], GRAIN_LEN_SAMPLES - FIR_MULTICORE_SPLIT_INDEX);
      int32_t result = 0;
      queue_add_blocking(&results_queue, &result);
    }
  }
}

/*
 * Copyright (c) 2010-2021 Arm Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * An unrolled version that can be inlined directly due to pico-sdk having problems with link time optimization
 *
 */
void arm_dot_prod_f32_inline(const float32_t *pSrcA, const float32_t *pSrcB, uint32_t blockSize, float32_t *result) {
  uint32_t blkCnt;      /* Loop counter */
  float32_t sum = 0.0f; /* Temporary return variable */

  /* Loop unrolling: Compute 4 outputs at a time */
  blkCnt = blockSize >> 2U;

  /* First part of the processing with loop unrolling. Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
  while (blkCnt > 0U) {
    /* C = A[0]* B[0] + A[1]* B[1] + A[2]* B[2] + .....+ A[blockSize-1]* B[blockSize-1] */

    /* Calculate dot product and store result in a temporary buffer. */
    sum += (*pSrcA++) * (*pSrcB++);

    sum += (*pSrcA++) * (*pSrcB++);

    sum += (*pSrcA++) * (*pSrcB++);

    sum += (*pSrcA++) * (*pSrcB++);

    /* Decrement loop counter */
    blkCnt--;
  }

  /* Loop unrolling: Compute remaining outputs */
  blkCnt = blockSize % 0x4U;

  while (blkCnt > 0U) {
    /* C = A[0]* B[0] + A[1]* B[1] + A[2]* B[2] + .....+ A[blockSize-1]* B[blockSize-1] */

    /* Calculate dot product and store result in a temporary buffer. */
    sum += (*pSrcA++) * (*pSrcB++);

    /* Decrement loop counter */
    blkCnt--;
  }

  /* Store result in destination buffer */
  *result = sum;
}

void iir_lpc_synthesis(float32_t *src, float32_t *dst, const float32_t *coeffs, float32_t *history_buffer,
                       uint32_t order, uint32_t blockSize) {
  // y[n] = x[n] - (a_1*y[n-1] + a_2*y[n-2] + ... + a_order*y[n-order])
  for (uint32_t i = 0; i < blockSize; i++) {
    // feedback = a[0]*y[n-1] + a[1]*y[n-2] + ... + a[order-1]*y[n-order]
    float32_t feedback;
    arm_dot_prod_f32_inline(&history_buffer[i], coeffs, LPC_ORDER, &feedback);

    dst[i] = history_buffer[order + i] = src[i] - feedback;
  }

  // Move the last order samples to the start of the history_buffer in a newest first manner to prepare for the next
  // function call
  for (uint32_t i = 0; i < order; i++) {
    history_buffer[i] = history_buffer[blockSize - i - 1];
  }
}

/* Public Functions */
void audio_proc_init() {
  arm_biquad_cascade_df2T_init_f32(&iir_dc_block_instance, 1, iir_dc_block_coeffs, iir_dc_block_state);
  arm_fir_init_f32(&fir_lpc_analysis_instance_core0, FIR_CORE0_NUM_TAPS, fir_lpc_analysis_coeffs,
                   fir_lpc_analysis_state_core0, FIR_MULTICORE_SPLIT_INDEX);
  arm_fir_init_f32(&fir_lpc_analysis_instance_core1, FIR_CORE1_NUM_TAPS, fir_lpc_analysis_coeffs,
                   fir_lpc_analysis_state_core1, GRAIN_LEN_SAMPLES - FIR_MULTICORE_SPLIT_INDEX);

  // Create trapezoidal taper window
  uint32_t single_edge_overlap_len = GRAIN_LEN_SAMPLES - STRIDE_SAMPLES - 1;
  for (int i = 0; i < GRAIN_LEN_SAMPLES; i++) {
    if (i < single_edge_overlap_len) {
      taper_window[i] = (float)i / single_edge_overlap_len;
    } else if (i >= GRAIN_LEN_SAMPLES - single_edge_overlap_len) {
      taper_window[i] = taper_window[GRAIN_LEN_SAMPLES - i - 1];
    } else {
      taper_window[i] = 1.0f;
    }
  }

  // Precompute interpolation indices and amplitudes
  for (int i = 0; i < GRAIN_LEN_SAMPLES; i++) {
    float32_t interp_index = (float32_t)i * PITCH_SHIFT_FACTOR;
    interpolation_indices[i] = (uint16_t)interp_index;
    interpolation_amps[i] = interp_index - interpolation_indices[i];
  }

  // Setup DMA channel for moving buffers around
  dma_write_rb_chan = dma_claim_unused_channel(true);
  dma_write_rb_config = dma_channel_get_default_config(dma_write_rb_chan);
  channel_config_set_transfer_data_size(&dma_write_rb_config, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_write_rb_config, true);
  channel_config_set_write_increment(&dma_write_rb_config, true);
  channel_config_set_ring(&dma_write_rb_config, true, 13);

  dma_read_rb_chan = dma_claim_unused_channel(true);
  dma_read_rb_config = dma_channel_get_default_config(dma_read_rb_chan);
  channel_config_set_transfer_data_size(&dma_read_rb_config, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_read_rb_config, true);
  channel_config_set_write_increment(&dma_read_rb_config, true);
  channel_config_set_ring(&dma_read_rb_config, false, 13);

  // Setup queues to send tasks to core 1
  queue_init(&autocorrelation_queue, sizeof(autocorr_job_t), 2);
  queue_init(&fir_lpc_queue, sizeof(uint32_t), 2);
  queue_init(&results_queue, sizeof(int32_t), 2);

  multicore_launch_core1(launch_core1);
}

void audio_process() {
  if (audio_mute) {
    rb_increment_read_index(&g_i2s_to_proc_buffer);
    int16_t *usb_buf = (int16_t *)rb_get_write_buffer(&g_proc_to_usb_buffer);
    memset(usb_buf, 0, AUDIO_PACKET_SIZE);
    rb_increment_write_index(&g_proc_to_usb_buffer);
    rb_reset(&x_concat);
    rb_reset(&output_fifo);
    memset(grain_overlap, 0, OVERLAP_LEN * sizeof(float32_t));
    return;
  }

  static uint32_t rng_state = 0x12345678; // Seed for RNG
  // Two buffers are needed because CMSIS-DSP filter functions are not inplace
  static float32_t processing_buf1[AUDIO_PACKET_SAMPLES];
  static float32_t processing_buf2[AUDIO_PACKET_SAMPLES];
  static float32_t temp_grain_buffer[GRAIN_LEN_SAMPLES];

  int16_t *audio_buf = (int16_t *)rb_get_read_buffer(&g_i2s_to_proc_buffer);
  int16_t *usb_buf = (int16_t *)rb_get_write_buffer(&g_proc_to_usb_buffer);

  arm_q15_to_float(audio_buf, processing_buf1, AUDIO_PACKET_SAMPLES);
  rb_increment_read_index(&g_i2s_to_proc_buffer);

  // filter DC component
  arm_biquad_cascade_df2T_f32(&iir_dc_block_instance, processing_buf1, processing_buf2, BLOCK_SIZE);

  write_to_ring_buffer_with_dma(&x_concat, processing_buf2, BLOCK_SIZE);
  // Granular Synthesis Processing if enough samples are available
  if (size(&x_concat) >= GRAIN_LEN_SAMPLES) {
    gpio_put(15, 1);
    write_to_grain_buffer_from_dma(&x_concat, input_grain_buffer, GRAIN_LEN_SAMPLES);

    // Let the first half be done by core 1
    autocorr_job_t autocorr_job = (autocorr_job_t){.start_delay = 0, .end_delay = LPC_ORDER >> 2};
    queue_add_blocking(&autocorrelation_queue, &autocorr_job);
    autocorrelate(input_grain_buffer, GRAIN_LEN_SAMPLES, autocorrelation_vector, (LPC_ORDER - (LPC_ORDER >> 2)),
                  LPC_ORDER);

    int32_t result;
    queue_remove_blocking(&results_queue, &result);

    float err;
    arm_levinson_durbin_f32(autocorrelation_vector, fir_lpc_analysis_coeffs, &err, LPC_ORDER);

    // forward filter grain using the computed coefficients
    memcpy(fir_lpc_analysis_state_core0, fir_global_history, (FIR_CORE0_NUM_TAPS - 1) * sizeof(float32_t));
    // This needs to change if the value can be negative. Some history will be taken from the input buffer
    uint32_t core1_histor_offset = FIR_MULTICORE_SPLIT_INDEX - (FIR_CORE0_NUM_TAPS - 1);
    memcpy(fir_lpc_analysis_state_core1, &input_grain_buffer[core1_histor_offset], (FIR_CORE1_NUM_TAPS - 1));
    uint32_t trigger = 0;
    queue_add_blocking(&fir_lpc_queue, &trigger);
    arm_fir_f32(&fir_lpc_analysis_instance_core0, input_grain_buffer, lpc_excitation, FIR_MULTICORE_SPLIT_INDEX);
    queue_remove_blocking(&results_queue, &result);
    memcpy(fir_global_history, &input_grain_buffer[GRAIN_LEN_SAMPLES - (FIR_CORE0_NUM_TAPS - 1)],
           (FIR_CORE0_NUM_TAPS - 1) * sizeof(float32_t));

    // Resample the grain
    for (int i = 0; i < GRAIN_LEN_SAMPLES; i++) {
      const uint16_t interp_idx = interpolation_indices[i];
      const float32_t interp_amp = interpolation_amps[i];
      const float32_t x0 = lpc_excitation[interp_idx];
      const float32_t x1 = lpc_excitation[interp_idx + 1];
      temp_grain_buffer[i] = x0 * (1.0f - interp_amp) + x1 * interp_amp;
    }

    // reverse filter the resampled and filtered grain
    iir_lpc_synthesis(temp_grain_buffer, lpc_excitation, fir_lpc_analysis_coeffs, iir_lpc_synthesis_state, LPC_ORDER,
                      GRAIN_LEN_SAMPLES);

    arm_mult_f32(lpc_excitation, taper_window, lpc_excitation, GRAIN_LEN_SAMPLES);
    arm_add_f32(lpc_excitation, grain_overlap, lpc_excitation, OVERLAP_LEN);
    memcpy(grain_overlap, &lpc_excitation[STRIDE_SAMPLES], OVERLAP_LEN * sizeof(float32_t));
    write_to_ring_buffer_with_dma(&output_fifo, lpc_excitation, STRIDE_SAMPLES);
    rb_increase_read_index(&x_concat, STRIDE_SAMPLES);
    gpio_put(15, 0);
  }

  if (size(&output_fifo) >= BLOCK_SIZE) {
    for (int i = 0; i < BLOCK_SIZE; i++) {
      processing_buf2[i] = *((float32_t *)rb_get_read_buffer(&output_fifo));
      rb_increment_read_index(&output_fifo);
    }
  } else {
    memset(processing_buf2, 0, BLOCK_SIZE * sizeof(float32_t));
  }

  // Prepare signal for USB transmission
  if (fabsf(audio_volume_multiplier - 1.0f) > 0.001f) {
    arm_scale_f32(processing_buf2, audio_volume_multiplier * 10, processing_buf2, BLOCK_SIZE);
  }
  arm_clip_f32(processing_buf2, processing_buf2, -1.0f, 1.0f, BLOCK_SIZE);
  for (int i = 0; i < AUDIO_PACKET_SAMPLES; i++) {
    usb_buf[i] = float_to_pcm16_dither(processing_buf2[i], &rng_state);
  }
  rb_increment_write_index(&g_proc_to_usb_buffer);
}
