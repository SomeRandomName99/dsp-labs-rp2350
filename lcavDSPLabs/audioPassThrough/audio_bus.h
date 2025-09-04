#ifndef AUDIO_BUS_H
#define AUDIO_BUS_H

#include "ring_buffer.h"

extern ring_buffer_t g_i2s_to_proc_buffer;
extern ring_buffer_t g_proc_to_usb_buffer;

#endif //AUDIO_BUS_H