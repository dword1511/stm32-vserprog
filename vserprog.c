#include <stdbool.h>
#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>

#include "usbcdc.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

#define PACKET_SIZE 64
#define BUFFER_SIZE 256

int main(void) {
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_AFIO);
  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, AFIO_MAPR_TIM2_REMAP_FULL_REMAP);

  /* Setup PA0 for the LED. */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
  gpio_set(GPIOA, GPIO0);

  /* Setup PB3 to pull up the D+ high. */
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO3);
  gpio_set(GPIOB, GPIO3);

  usbcdc_init();

  gpio_clear(GPIOA, GPIO0);


  /* Wait 500ms for USB setup to complete before trying to send anything. */


  /* The loop. */
  int i;
  while (true) {
    for (i = 0; i < 10000000; i ++) {
      asm("nop");
    }
    gpio_toggle(GPIOA, GPIO0);
    usbcdc_write("HELLO!\r\n", 8);
  }

  return 0;
}

/* Interrupts */
