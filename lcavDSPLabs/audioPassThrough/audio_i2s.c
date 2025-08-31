#include "hardware/dma.h"
#include "audio_i2s.h"
#include "audio_usb.h"
#include "audioPassThrough.pio.h"

/* Constants and Macros*/
#define BCLK_PIN 2
#define WS_PIN   BCLK_PIN+1
#define DATA_PIN 4

/* Private Global Variables */
static PIO pio;
static uint sm;
volatile static uint8_t i2s_buffer[AUDIO_PACKET_SIZE]; // double buffering should be  implemented here actually
// we should fill a buffer then process it and copy the final answer to the audio_usb buffer to be sent 
// Each time the DMA channel finished copying a buffer's worth of data, we switch to the next buffer
// Hopefully there is enough time for all our processing needs. Alternatively we can incrfease the size of
// our buffer to give the cpu some time. Not sure how this would work though because we would reache some sort
// steady state in the end. I have to try it out first.

// But to start with let's implement DMA double buffering, maybe make it extensible, so using 4 or 8 or 16(a power of 2) is
// pretty easy.

/* Public Functions */
void audio_i2s_init(PIO pio_, uint sm_){
  pio = pio_;
  sm = sm_;
  uint offset = pio_add_program(pio, &audioPassThrough_program);
  audioPassThrough_program_init(pio, sm, offset, BCLK_PIN, WS_PIN, DATA_PIN);
}

void audio_i2s_usb_dma_init() {
  uint dataChan = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(dataChan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
  channel_config_set_irq_quiet(&c, true);

  // TODO: Check the number of writes done, I think that something is wrong there
  dma_channel_configure(dataChan, &c,
    i2s_buffer,
    &pio0_hw->rxf[sm],
    AUDIO_PACKET_SAMPLES,
    true // TODO: set this correctly
  );

  dma_channel_wait_for_finish_blocking(dataChan);
  uint test = 1;
}