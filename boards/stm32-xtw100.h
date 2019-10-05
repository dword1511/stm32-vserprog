#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/*
 * XTW100 flash programmer
 * can be bought from https://detail.tmall.com/item.htm?id=40083335389
 * To flash it, desolder the zero-ohm resistor R6, then program it
 * with the ISP1 pinouts
 */

#define BOARD_USE_DEBUG_PINS_AS_GPIO false

#define BOARD_RCC_LED                RCC_GPIOB
#define BOARD_PORT_LED               GPIOB
#define BOARD_PIN_LED                GPIO7
#define BOARD_LED_HIGH_IS_BUSY       true

#define BOARD_RCC_USB_PULLUP         RCC_GPIOB
#define BOARD_PORT_USB_PULLUP        GPIOB
#define BOARD_PIN_USB_PULLUP         GPIO8
#define BOARD_USB_HIGH_IS_PULLUP     true

/* Currently you can only use SPI1, since it has highest clock. */

#endif /* __BOARD_H__ */
