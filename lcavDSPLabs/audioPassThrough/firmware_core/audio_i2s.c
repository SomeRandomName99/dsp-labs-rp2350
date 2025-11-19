#include "hardware/dma.h"
#include "audio_i2s.h"
#include "audio_usb.h"
#include "audio_bus.h"
#include "firmware_dsp.pio.h"

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
void dma_handler() {
  dma_hw->ints0 = 1u << dma_data_chan;

  void *next_buf = rb_get_write_buffer(&g_i2s_to_proc_buffer);
  rb_increment_write_index(&g_i2s_to_proc_buffer);
  bool trigger = true;
  dma_channel_set_write_addr(dma_data_chan, next_buf, trigger);
}

/* Public Functions */
void audio_i2s_init(PIO pio_, uint sm_){
  pio = pio_;
  sm = sm_;
  uint offset = pio_add_program(pio, &firmware_dsp_program);
  hard_assert(offset != -1);
  firmware_dsp_program_init(pio, sm, offset, BCLK_PIN, WS_PIN, DATA_PIN);
}

void audio_i2s_usb_dma_init() {
  dma_data_chan = dma_claim_unused_channel(true);

  dma_channel_config c = dma_channel_get_default_config(dma_data_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
  dma_channel_set_irq0_enabled(dma_data_chan, true);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_handler); 
  irq_set_enabled(DMA_IRQ_0, true);

  bool trigger = true;
  void *write_buf = rb_get_write_buffer(&g_i2s_to_proc_buffer);
  rb_increment_write_index(&g_i2s_to_proc_buffer);
  dma_channel_configure(dma_data_chan, &c,
    write_buf,
    &pio0_hw->rxf[sm],
    AUDIO_PACKET_SAMPLES/2, // each sample is 16 bits and we move 32-bits per transfer
    trigger
  );
}
