#ifndef AUDIO_BUS_H
#define AUDIO_BUS_H

#include "ring_buffer.h"
#include "audio_usb.h"

/* Global State Constants */
#define SAMPLE_RATE            48000
#define AUDIO_BYTES_PER_SAMPLE 2 // 16-bit
#define AUDIO_CHANNELS         1
#define AUDIO_SAMPLES_PER_MS   (SAMPLE_RATE / 1000)
#define AUDIO_PACKET_SAMPLES   (AUDIO_SAMPLES_PER_MS * AUDIO_CHANNELS)
#define AUDIO_PACKET_SIZE      AUDIO_PACKET_SAMPLES * AUDIO_BYTES_PER_SAMPLE

/* Global State Variables */
extern ring_buffer_t g_i2s_to_proc_buffer;
extern ring_buffer_t g_proc_to_usb_buffer;
extern bool audio_mute;
extern float audio_volume_multiplier; // [0.0, 1.0]

#endif //AUDIO_BUS_H