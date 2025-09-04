#include "hardware/dma.h"
#include "audio_i2s.h"
#include "audio_usb.h"
#include "audio_bus.h"
#include "audioPassThrough.pio.h"

/* Constants and Macros*/
#define BCLK_PIN 2
#define WS_PIN   BCLK_PIN+1
#define DATA_PIN 4

#define BUFFER_COUNT 4 // Must be a power of 2 due to DMA Auto-Wrapping

/* Private Global Variables */
static PIO pio;
static uint sm;
static uint dma_data_chan;

/* Private Helper Functions */
static void dma_handler() {
  dma_hw->ints0 = 1u << dma_data_chan;

  uint8_t *next_buf = rb_get_write_buffer(&g_i2s_to_proc_buffer);
  uint8_t *next_buf = rb_is_get_write_buffer(&g_i2s_to_proc_buffer);
  bool trigger = true;
  dma_channel_set_read_addr(dma_data_chan, next_buf, trigger);

}

/* Public Functions */
void audio_i2s_init(PIO pio_, uint sm_){
  pio = pio_;
  sm = sm_;
  uint offset = pio_add_program(pio, &audioPassThrough_program);
  hard_assert(offset != -1);
  audioPassThrough_program_init(pio, sm, offset, BCLK_PIN, WS_PIN, DATA_PIN);
}

void audio_i2s_usb_dma_init() {
  uint data_chan = dma_claim_unused_channel(true);

  dma_channel_config c = dma_channel_get_default_config(data_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
  channel_config_set_irq_quiet(&c, true);
  dma_channel_set_irq0_enabled(data_chan, true);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_handler); 
  irq_set_enabled(DMA_IRQ_0, true);

  bool trigger = true;
  dma_channel_configure(data_chan, &c,
    g_i2s_to_proc_buffer.buffer, // Set by control channel
    &pio0_hw->rxf[sm],
    AUDIO_PACKET_SAMPLES/2, // each sample is 16 bits and we move 32-bits per transfer
    trigger
  );
}
