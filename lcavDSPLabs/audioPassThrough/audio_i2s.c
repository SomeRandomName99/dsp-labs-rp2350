#include "hardware/dma.h"
#include "audio_i2s.h"
#include "audio_usb.h"
#include "audioPassThrough.pio.h"

/* Constants and Macros*/
#define BCLK_PIN 2
#define WS_PIN   BCLK_PIN+1
#define DATA_PIN 4

#define BUFFER_COUNT 4 // Must be a power of 2 due to DMA Auto-Wrapping

/* Private Global Variables */
static PIO pio;
static uint sm;

volatile uint16_t i2s_buffer[AUDIO_PACKET_SAMPLES*(BUFFER_COUNT-1)]; // The last control block is null trigger

static volatile uint16_t* dma_control_blocks[BUFFER_COUNT];

// double buffering should be  implemented here actually
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
  hard_assert(offset != -1);
  audioPassThrough_program_init(pio, sm, offset, BCLK_PIN, WS_PIN, DATA_PIN);
}

void audio_i2s_usb_dma_init() {
  uint data_chan = dma_claim_unused_channel(true);
  uint cntrl_chan = dma_claim_unused_channel(true);

  dma_channel_config c = dma_channel_get_default_config(data_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
  channel_config_set_chain_to(&c, cntrl_chan);
  channel_config_set_irq_quiet(&c, true);

  dma_channel_configure(data_chan, &c,
    NULL, // Set by control channel
    &pio0_hw->rxf[sm],
    AUDIO_PACKET_SAMPLES/2, // each sample is 16 bits and we move 32-bits per transfer
    false
  );

  for(int i = 0; i < BUFFER_COUNT; i++){
    if(i == BUFFER_COUNT-1){
      dma_control_blocks[i] = NULL;
      continue;
    }
    dma_control_blocks[i] = &i2s_buffer[i*AUDIO_PACKET_SAMPLES];
  }

  c = dma_channel_get_default_config(cntrl_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  channel_config_set_ring(&c, false, 4);
  channel_config_set_irq_quiet(&c, true);

  dma_channel_configure(cntrl_chan, &c,
    &dma_hw->ch[data_chan].al2_write_addr_trig,
    dma_control_blocks,
    1,
    true
  );
  // dma_channel_wait_for_finish_blocking(cntrl_chan);
  // uint test = 1;

  // Maybe double the buffers and let the cpu processing IRQ trigger the control channels
  // Which triggers the data channel and they only write to half the buffer. hmmm interesting

  // The data channel will assert its IRQ flag when it gets a null trigger,
  // indicating the end of the control block list. We're just going to wait
  // for the IRQ flag instead of setting up an interrupt handler.
  while (!(dma_hw->intr & 1u << data_chan))
      tight_loop_contents();
  uint test = 1;
  dma_hw->ints0 = 1u << data_chan;
}