#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define BOARD_USE_DEBUG_PINS_AS_GPIO false

#define BOARD_RCC_LED                RCC_GPIOB
#define BOARD_PORT_LED               GPIOB
#define BOARD_PIN_LED                GPIO1
#define BOARD_LED_HIGH_IS_BUSY       false /* We only have IDLE LED on it, which is active high. */

#define BOARD_RCC_USB_PULLUP         RCC_GPIOB
#define BOARD_PORT_USB_PULLUP        GPIOB
#define BOARD_PIN_USB_PULLUP         GPIO9
#define BOARD_USB_HIGH_IS_PULLUP     false /* Active low */

/* Currently you can only use SPI1, since it has highest clock. */

#endif /* __BOARD_H__ */
