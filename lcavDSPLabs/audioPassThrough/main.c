#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "audioPassThrough.pio.h"
#include "audio_usb.h"

#define BCLK_PIN 2
#define WS_PIN   BCLK_PIN+1
#define DATA_PIN 4

#define LED_DELAY_MS 500

#define DBG_LOOP_PIN 14
#define DBG_AUDIO_PIN 15

void toggleLED();
void debugGPIO_init();

int main() {
  /* Init functions */
  stdio_uart_init();
  cyw43_arch_init();
  board_init();
  tusb_init();
  audio_usb_init();
  // debugGPIO_init();

  /* PIO setup */
  PIO pio = pio0;
  uint sm = 0;
  uint offset = pio_add_program(pio, &audioPassThrough_program);
  audioPassThrough_program_init(pio, sm, offset, BCLK_PIN, WS_PIN, DATA_PIN);

  // uint32_t clkdiv_reg = pio->sm[sm].clkdiv;
  // uint32_t clkdiv_int = clkdiv_reg >> 16;
  // uint32_t clkdiv_frac = (clkdiv_reg >> 8) & 0xFF;
  // printf("SM%d Clock Divisor: %lu.%lu\n", sm, clkdiv_int, clkdiv_frac);

  /* DMA Test */
  uint dataChan = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(dataChan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
  channel_config_set_irq_quiet(&c, true);

  uint32_t buffer[16];
  for(int i = 0; i < 16; i++){
    buffer[i] = 0xFFFFFFFF;
  }
  dma_channel_configure( dataChan, &c,
    buffer,
    &pio0_hw->rxf[sm],
    16,
    true
  );

  dma_channel_wait_for_finish_blocking(dataChan);

  while(1){
    tud_task();
    audio_task();
    toggleLED();
  }
}

/* Local function implementation */
void toggleLED(void){
  static bool isOn = false;

  static uint32_t toggleTime = 0;
  uint32_t curTime =  board_millis();
  bool toggle = (curTime - toggleTime) >= 500;

  if(toggle){
    isOn = !isOn;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, isOn);
    toggleTime = curTime;
  }
}

void debugGPIO_init(){
  gpio_init(DBG_LOOP_PIN);
  gpio_init(DBG_AUDIO_PIN);

  gpio_set_dir(DBG_LOOP_PIN, GPIO_OUT);
  gpio_set_dir(DBG_AUDIO_PIN, GPIO_OUT);
}