#include <math.h>

#include "dsp/filtering_functions.h"
#include "hardware/dma.h"

#include "arm_math.h"

#include "audio_bus.h"
#include "audio_usb.h"
#include "ring_buffer.h"

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

/* State Variables */
// https://arm-software.github.io/CMSIS-DSP/latest/group__BiquadCascadeDF2T.html
static float32_t iir_dc_block_coeffs[5] = {
    1, -1, 0,  // b0, b1, b2
    0.9993f, 0 // a1, a2
};
static float32_t iir_dc_block_state[2];
static arm_biquad_cascade_df2T_instance_f32 iir_dc_block_instance = {
    .numStages = 1, .pCoeffs = iir_dc_block_coeffs, .pState = iir_dc_block_state};

static float32_t iir_lpc_synthesis_coeffs[5 * IIR_LPC_STAGES];
static float32_t iir_lpc_synthesis_state[LPC_ORDER];
static arm_biquad_cascade_df2T_instance_f32 iir_lpc_synthesis_instance = {
    .numStages = IIR_LPC_STAGES, .pCoeffs = iir_lpc_synthesis_coeffs, .pState = iir_lpc_synthesis_state};

static float32_t taper_window[GRAIN_LEN_SAMPLES];          // 4KB
static uint16_t interpolation_indices[GRAIN_LEN_SAMPLES];  // 2KB
static float32_t interpolation_amps[GRAIN_LEN_SAMPLES];    // 4KB
static float32_t grain_overlap[OVERLAP_LEN] = {0};         // 0.4KB
rb_init_static(x_concat, (1 << 11), sizeof(float32_t));    // 8KB Supports grains of up to 42ms
rb_init_static(output_fifo, (1 << 11), sizeof(float32_t)); // 8KB
static float32_t input_grain_buffer[GRAIN_LEN_SAMPLES];
static float32_t autocorrelation_vector[2 * GRAIN_LEN_SAMPLES - 1];
static float32_t lpc_excitation[GRAIN_LEN_SAMPLES];
static float32_t fir_lpc_analysis_coeffs[GRAIN_LEN_SAMPLES];
static arm_fir_instance_f32 fir_lpc_analysis_instance;
static float32_t fir_lpc_analysis_state[GRAIN_LEN_SAMPLES + LPC_ORDER];

static uint32_t dma_data_chan;
static dma_channel_config dma_config;

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
  uint16_t num_writes_before_wrap = rb_get_num_contiguous_writes(rb);
  if (num_writes_before_wrap < length) {
    dma_channel_configure(dma_data_chan, &dma_config, rb_get_write_buffer(rb), data, num_writes_before_wrap, true);
    dma_channel_wait_for_finish_blocking(dma_data_chan);
    rb_increase_write_index(rb, num_writes_before_wrap);
    dma_channel_configure(dma_data_chan, &dma_config, rb_get_write_buffer(rb), &data[num_writes_before_wrap],
                          length - num_writes_before_wrap, true);
    dma_channel_wait_for_finish_blocking(dma_data_chan);
    rb_increase_write_index(rb, length - num_writes_before_wrap);
  } else {
    dma_channel_configure(dma_data_chan, &dma_config, rb_get_write_buffer(rb), data, length, true);
    dma_channel_wait_for_finish_blocking(dma_data_chan);
    rb_increase_write_index(rb, length);
  }
}

static inline void write_to_grain_buffer_from_dma(ring_buffer_t *rb, float32_t *data, uint16_t length) {
  assert(size(rb) >= length);
  uint16_t num_reads_before_wrap = rb_get_num_contiguous_reads(rb);
  if (num_reads_before_wrap < length) {
    dma_channel_configure(dma_data_chan, &dma_config, rb_get_write_buffer(rb), data, num_reads_before_wrap, true);
    dma_channel_wait_for_finish_blocking(dma_data_chan);
    dma_channel_configure(dma_data_chan, &dma_config, rb_get_write_buffer(rb), &data[num_reads_before_wrap],
                          length - num_reads_before_wrap, true);
    dma_channel_wait_for_finish_blocking(dma_data_chan);
  } else {
    dma_channel_configure(dma_data_chan, &dma_config, rb_get_write_buffer(rb), data, length, true);
    dma_channel_wait_for_finish_blocking(dma_data_chan);
  }
}

/* Public Functions */

void audio_proc_init() {
  arm_biquad_cascade_df2T_init_f32(&iir_dc_block_instance, 1, iir_dc_block_coeffs, iir_dc_block_state);
  arm_fir_init_f32(&fir_lpc_analysis_instance, LPC_ORDER + 1, fir_lpc_analysis_coeffs, fir_lpc_analysis_state,
                   GRAIN_LEN_SAMPLES);
  arm_biquad_cascade_df2T_init_f32(&iir_lpc_synthesis_instance, IIR_LPC_STAGES, iir_lpc_synthesis_coeffs,
                                   iir_lpc_synthesis_state);

  // Initialize the coedfficients, mostly useful for b0, b1, b2 which will not change even when toe coefficients are
  // updated
  for (uint32_t s = 0; s < IIR_LPC_STAGES; ++s) {
    const uint32_t base = 5 * s;
    iir_lpc_synthesis_coeffs[base + 0] = 1.0f; // b0
    iir_lpc_synthesis_coeffs[base + 1] = 0.0f; // b1
    iir_lpc_synthesis_coeffs[base + 2] = 0.0f; // b2
    iir_lpc_synthesis_coeffs[base + 3] = 0.0f; // a1
    iir_lpc_synthesis_coeffs[base + 4] = 0.0f; // a2
  }

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
  dma_data_chan = dma_claim_unused_channel(true);
  dma_config = dma_channel_get_default_config(dma_data_chan);
  channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_config, true);
  channel_config_set_write_increment(&dma_config, true);
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
    // arm_correlate_f32(input_grain_buffer, GRAIN_LEN_SAMPLES, input_grain_buffer, GRAIN_LEN_SAMPLES,
    //                   autocorrelation_vector);
    float err;
    // arm_levinson_durbin_f32(&autocorrelation_vector[GRAIN_LEN_SAMPLES - 1], fir_lpc_analysis_coeffs, &err, LPC_ORDER);

    // Update the iir filter's coefficients
    for (uint32_t s = 0; s < IIR_LPC_STAGES; ++s) {
      const float32_t a1 = fir_lpc_analysis_coeffs[2 * s + 0];
      const float32_t a2 = fir_lpc_analysis_coeffs[2 * s + 1];
      const uint32_t base = 5 * s;
      iir_lpc_synthesis_coeffs[base + 3] = a1;
      iir_lpc_synthesis_coeffs[base + 4] = a2;
    }

    // forward filter grain using the computed coefficients
    // arm_fir_f32(&fir_lpc_analysis_instance, input_grain_buffer, lpc_excitation, GRAIN_LEN_SAMPLES);

    for (int i = 0; i < GRAIN_LEN_SAMPLES; i++) {
      const uint16_t interp_idx = interpolation_indices[i];
      const float32_t interp_amp = interpolation_amps[i];
      const float32_t x0 = lpc_excitation[interp_idx];
      const float32_t x1 = lpc_excitation[interp_idx + 1];
      temp_grain_buffer[i] = x0 * (1.0f - interp_amp) + x1 * interp_amp;
    }

    // reverse filter the resampled and filtered grain
    // arm_biquad_cascade_df2T_f32(&iir_lpc_synthesis_instance, temp_grain_buffer, lpc_excitation, GRAIN_LEN_SAMPLES);

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
