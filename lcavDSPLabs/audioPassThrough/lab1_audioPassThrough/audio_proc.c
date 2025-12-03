#include <math.h>

#include "arm_math.h"

#include "audio_usb.h"
#include "audio_bus.h"

/* Constants and Macros */
#define BLOCK_SIZE AUDIO_PACKET_SAMPLES
#define NUM_TAPS 2

// https://arm-software.github.io/CMSIS-DSP/latest/group__FIR.html
static float32_t fir_dc_removal_coeffs[NUM_TAPS] = {-1.0f, 1.0f};
static float32_t fir_dc_removal_state[NUM_TAPS*BLOCK_SIZE-1];
static arm_fir_instance_f32 fir_dc_removal_instance;

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

static inline float rand_uniform(uint32_t *state) {
  // 24 bits are extracted because that is the precision if an f32 mantissa
  return (fast_rand(state) & 0xFFFFFF) / 16777216.0f; // 2^24
}

static inline int32_t fast_lrintf_to_nearest(float f) {
  int32_t result;
  asm (
    "vcvtn.s32.f32 %1, %1\n"
    "vmov %0, %1\n"
    :"=r"(result)
    :"w"(f)
  );
  return result;
}

static inline int16_t float_to_pcm16_dither(float x, uint32_t *rng_state) {
  // TPDF dithering
  float dither = (rand_uniform(rng_state) - rand_uniform(rng_state)) * (1.0f / 32768.0f);

  x += dither;

  if (x >= 1.0f) x = 1.0f;
  if (x < -1.0f) x = -1.0f;

  return fast_lrintf_to_nearest(x * 32767.0f); // we scale back to full range PCM16 here
}

// Convert PCM16 to float in range [-1.0, 1.0]
static inline float pcm16_to_float(int16_t x) {
  // https://discuss.pytorch.org/t/mechanism-of-represneting-pcm16-by-float32/141236
  return (x < 0) ? (x / 32768.0f) : (x / 32767.0f); 
}

static inline void audio_process_mute() {
  rb_increment_read_index(&g_i2s_to_proc_buffer);
  int16_t *usb_buf   = (int16_t *) rb_get_write_buffer(&g_proc_to_usb_buffer);
  memset(usb_buf, 0, AUDIO_PACKET_SIZE);
  rb_increment_write_index(&g_proc_to_usb_buffer);
}

/* Public Functions */

void audio_proc_init() {
  arm_fir_init_f32(&fir_dc_removal_instance, NUM_TAPS, fir_dc_removal_coeffs, fir_dc_removal_state, BLOCK_SIZE);
}

void audio_process(){
  if(audio_mute){
    audio_process_mute();
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
  rb_increment_read_index(&g_i2s_to_proc_buffer);

  // Signal Conditioning
  arm_fir_f32(&fir_dc_removal_instance, processing_buf1, processing_buf2, BLOCK_SIZE);
  if(fabsf(audio_volume_multiplier - 1.0f) > 0.001f){
    arm_scale_f32(processing_buf2, audio_volume_multiplier, processing_buf2, BLOCK_SIZE);
  }

  for(int i = 0; i < AUDIO_PACKET_SAMPLES; i++){
    usb_buf[i] = float_to_pcm16_dither(processing_buf2[i], &rng_state);
  }
  rb_increment_write_index(&g_proc_to_usb_buffer);
}