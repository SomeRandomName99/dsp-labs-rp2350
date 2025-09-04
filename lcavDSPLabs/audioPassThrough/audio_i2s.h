
#ifndef AUDIO_I2S_H
#define AUDIO_I2S_H

#include "hardware/pio.h"
#include "ring_buffer.h"

void audio_i2s_init(PIO pio, uint sm);
void audio_i2s_usb_dma_init();

#endif // AUDIO_I2S_H