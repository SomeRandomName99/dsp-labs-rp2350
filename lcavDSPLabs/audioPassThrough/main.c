#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "audioPassThrough.pio.h"
#include "audio_usb.h"
#include "audio_i2s.h"
#include "audio_proc.h"
#include "ring_buffer.h"

#define LED_DELAY_MS 500

#define DBG_LOOP_PIN 14
#define DBG_AUDIO_PIN 15

void toggleLED();
void debugGPIO_init();


rb_init_shared(g_i2s_to_proc_buffer, 3, AUDIO_PACKET_SIZE);
rb_init_shared(g_proc_to_usb_buffer, 3, AUDIO_PACKET_SIZE);

int main() {
  /* Init functions */
  stdio_uart_init();
  cyw43_arch_init();
  board_init();
  tusb_init();

  debugGPIO_init();
  gpio_put(15, 1); // TODO: Remove after debugging
  sleep_ms(100);

  audio_usb_init();
  audio_i2s_init(pio0, 0);
  audio_i2s_usb_dma_init();

  while(1){
    tud_task();
    if(!rb_is_empty(&g_i2s_to_proc_buffer)){
      audio_process();
      audio_task();
    }
    toggleLED();
  }
}

/* Local function implementation */
void toggleLED(void){
  static bool is_on = false;

  static uint32_t toggleTime = 0;
  uint32_t curTime =  board_millis();
  bool toggle = (curTime - toggleTime) >= 500;

  if(toggle){
    is_on = !is_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, is_on);
    toggleTime = curTime;
  }
}

void debugGPIO_init(){
  gpio_init(DBG_LOOP_PIN);
  gpio_init(DBG_AUDIO_PIN);

  gpio_set_dir(DBG_LOOP_PIN, GPIO_OUT);
  gpio_set_dir(DBG_AUDIO_PIN, GPIO_OUT);
}