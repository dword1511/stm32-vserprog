#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <libopencm3/cm3/nvic.h>


#define BOARD_USE_DEBUG_PINS_AS_GPIO false /* We don't have to disable SWD to access GPIO's on these boards */

#define BOARD_RCC_LED                RCC_GPIOB
#define BOARD_PORT_LED               GPIOB
#define BOARD_PIN_LED                GPIO7
#define BOARD_LED_HIGH_IS_BUSY       true

#define BOARD_RCC_USB_PULLUP         RCC_GPIOB
#define BOARD_PORT_USB_PULLUP        GPIOB
#define BOARD_PIN_USB_PULLUP         GPIO8
#define BOARD_USB_HIGH_IS_PULLUP     true

/* Currently you can only use SPI1, since it has highest clock. */

#define HAS_BOARD_INIT 1
void board_init (void) {
// Enable power LED (PB6)
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);
  gpio_set(GPIOB, GPIO6);
// Toggle power to the 25xx flash chip
// PB2 + PB1 serve as GND supply, GPIO_50_MHZ to increase drive current
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1|GPIO2);
  gpio_clear(GPIOB, GPIO1|GPIO2);
// Set #WP and #HOLD high (PA2|PA3)
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2|GPIO3);
  gpio_set(GPIOA, GPIO2|GPIO3);
}

#endif /* __BOARD_H__ */
