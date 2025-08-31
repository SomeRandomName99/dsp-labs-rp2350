#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "audioPassThrough.pio.h"
#include "audio_usb.h"
#include "audio_i2s.h"

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
  audio_i2s_init(pio0, 0);
  audio_i2s_usb_dma_init();
  // debugGPIO_init();

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