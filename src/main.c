#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define LED_PIN GPIO4

static void gpio_setup()
{
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

int main()
{
  int i;

  gpio_setup();
  for (;;) {
    gpio_toggle(GPIOA, LED_PIN);
    for (i = 0; i < 1000000; i++) {
      __asm__("nop");
    }
  }

  return 0;
}
