#include <math.h>

#include "arm_math.h"

#include "audio_usb.h"
#include "audio_bus.h"

/* Constants and Macros */
#define BLOCK_SIZE AUDIO_PACKET_SAMPLES
#define GRAIN_LEN_MS 20
#define GRAIN_OVERLAP_RATIO 0.2f
#define PITCH_SHIFT_FACTOR 0.7f
#define GRAIN_LEN_SAMPLES (AUDIO_SAMPLES_PER_MS * GRAIN_LEN_MS)
#define STRIDE ((GRAIN_LEN_SAMPLES - (int)(GRAIN_LEN_SAMPLES * GRAIN_OVERLAP_RATIO)/2)-1)
#define OVERLAP_LEN (GRAIN_LEN_SAMPLES - STRIDE)

/* State Variables */
// https://arm-software.github.io/CMSIS-DSP/latest/group__BiquadCascadeDF2T.html
static float32_t iir_dc_block_coeffs[5] = {
  1, -1, 0,  // b0, b1, b2
  0.9993f, 0 // a1, a2
};
static float32_t iir_dc_block_state[2];  
static arm_biquad_cascade_df2T_instance_f32 iir_dc_block_instance = {
  .numStages = 1,
  .pCoeffs = iir_dc_block_coeffs,
  .pState = iir_dc_block_state
};

static float32_t taper_window[GRAIN_LEN_SAMPLES];

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

  if (x >= 1.0f) x = 1.0f;
  if (x < -1.0f) x = -1.0f;

  // round to the nearest integer using 1 hardware instruction
  int32_t result;
  asm (
    "vcvtn.s32.f32 %1, %1\n"
    "vmov %0, %1\n"
    :"=r"(result)
    :"w"(x * 32767.0f) // we scale back to full range PCM16 here
  );
  return result;

  return (int16_t)result; 
}

// Convert PCM16 to float in range [-1.0, 1.0]
static inline float pcm16_to_float(int16_t x) {
  // https://discuss.pytorch.org/t/mechanism-of-represneting-pcm16-by-float32/141236
  return (x < 0) ? (x / 32768.0f) : (x / 32767.0f); 
}

/* Public Functions */
#if LAB_ID == 4

void audio_proc_init() {
  arm_biquad_cascade_df2T_init_f32(&iir_dc_block_instance, 1, iir_dc_block_coeffs, iir_dc_block_state);

  // Create trapezoidal taper window
  uint32_t single_edge_overlap_len = (GRAIN_LEN_SAMPLES * GRAIN_OVERLAP_RATIO)/2;
  for(int n = 0; n < GRAIN_LEN_SAMPLES; n++){
    if(n < single_edge_overlap_len){
      taper_window[n] = (float)n / single_edge_overlap_len;
    } else if(n >= GRAIN_LEN_SAMPLES - single_edge_overlap_len){
      taper_window[n] = taper_window[GRAIN_LEN_SAMPLES - n - 1];
    } else {
      taper_window[n] = 1.0f;
    }
  }
}

void audio_process(){
  if(audio_mute){
    rb_increase_read_index(&g_i2s_to_proc_buffer);
    int16_t *usb_buf   = (int16_t *) rb_get_write_buffer(&g_proc_to_usb_buffer);
    memset(usb_buf, 0, AUDIO_PACKET_SIZE);
    rb_increase_write_index(&g_proc_to_usb_buffer);
    return;
  }

  static uint32_t rng_state = 0x12345678; // Seed for RNG
  // Two buffers are needed because CMSIS-DSP filter functions are not inplace
  static float processing_buf1[AUDIO_PACKET_SAMPLES];
  static float processing_buf2[AUDIO_PACKET_SAMPLES];

  int16_t *audio_buf = (int16_t *) rb_get_read_buffer(&g_i2s_to_proc_buffer);
  int16_t *usb_buf   = (int16_t *) rb_get_write_buffer(&g_proc_to_usb_buffer);

  for(int i = 0; i < AUDIO_PACKET_SAMPLES; i++){
    processing_buf1[i] = pcm16_to_float(audio_buf[i]);
  }
  rb_increase_read_index(&g_i2s_to_proc_buffer);

  // filter DC component
  arm_biquad_cascade_df2T_f32(&iir_dc_block_instance, processing_buf1, processing_buf2, BLOCK_SIZE);

  // Prepare signal for USB transmission
  if(fabsf(audio_volume_multiplier - 1.0f) > 0.001f){
    arm_scale_f32(processing_buf2, audio_volume_multiplier*4, processing_buf2, BLOCK_SIZE);
  }
  for(int i = 0; i < AUDIO_PACKET_SAMPLES; i++){
    usb_buf[i] = float_to_pcm16_dither(processing_buf2[i], &rng_state);
  }
  rb_increase_write_index(&g_proc_to_usb_buffer);
}
#endif // LAB_ID == 4