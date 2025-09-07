#ifndef AUDIO_BUS_H
#define AUDIO_BUS_H

#include "ring_buffer.h"
#include "audio_usb.h"

extern ring_buffer_t g_i2s_to_proc_buffer;
extern ring_buffer_t g_proc_to_usb_buffer;
extern bool audio_mute;
extern float audio_volume_multiplier; // [0.0, 1.0]

#endif //AUDIO_BUS_H