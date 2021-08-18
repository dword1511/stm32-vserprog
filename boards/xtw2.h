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

#define GD32F103_8MHZ

/*
 * The XTW2 is like the XTW100, but with a GD32F103 and a single pinout for both SPI and I2C.
 * The GD32F103 is mostly the same as an STM32F103, but its SPI peripherals can all do 18MHz officially,
 * and SPI1 at least has been seen doing 54MHz in the wild. We don't care about I2C, so, whatever.
 * VCC and GND are hooked straight to the regulator/board, which is an improvement over the XTW100.
 */

#define HAS_BOARD_INIT 1
void board_init (void) {
  // Set #WP and #HOLD high (PA2|PA3)
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2|GPIO3);
  gpio_set(GPIOA, GPIO2|GPIO3);
}

#endif /* __BOARD_H__ */
