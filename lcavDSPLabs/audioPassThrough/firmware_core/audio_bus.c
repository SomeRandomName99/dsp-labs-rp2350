
#include "ring_buffer.h"
#include "audio_bus.h"

rb_init_shared(g_i2s_to_proc_buffer, 4, AUDIO_PACKET_SIZE);
rb_init_shared(g_proc_to_usb_buffer, 4, AUDIO_PACKET_SIZE);
bool audio_mute = false;
float audio_volume_multiplier = 1.0f;

