#include "hardware/pio.h"
#include "audioPassThrough.pio.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "stdio.h"
#include "audio_usb.h"

#define BCLK_PIN 2
#define WS_PIN   3
#define DATA_PIN 4

#define LED_DELAY_MS 500

int main() {
  stdio_uart_init();
  cyw43_arch_init();
  board_init();
  tusb_init();

  audio_usb_init();

  PIO pio;
  uint sm;
  uint offset;

  bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&audioPassThrough_program, &pio, &sm, &offset, BCLK_PIN, 3, true);
  hard_assert(success);
  printf("PIO setup success: %d\n", success);
  printf("Using PIO%d, SM%d, offset%d\n", pio_get_index(pio), sm, offset);
  audioPassThrough_program_init(pio, sm, offset, BCLK_PIN, WS_PIN, DATA_PIN);

  uint32_t clkdiv_reg = pio->sm[sm].clkdiv;
  uint32_t clkdiv_int = clkdiv_reg >> 16;
  uint32_t clkdiv_frac = (clkdiv_reg >> 8) & 0xFF;
  printf("SM%d Clock Divisor: %lu.%lu\n", sm, clkdiv_int, clkdiv_frac);

  while(1){
    // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
    // sleep_ms(LED_DELAY_MS);
    // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
    // sleep_ms(LED_DELAY_MS);
    tud_task();
  }
}