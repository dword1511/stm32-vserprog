#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define BOARD_USE_DEBUG_PINS_AS_GPIO true /* We have to disable JTAG / SWD to access PB3 */

#define BOARD_RCC_LED                RCC_GPIOA
#define BOARD_PORT_LED               GPIOA
#define BOARD_PIN_LED                GPIO0
#define BOARD_LED_HIGH_IS_BUSY       true

#define BOARD_RCC_USB_PULLUP         RCC_GPIOB
#define BOARD_PORT_USB_PULLUP        GPIOB
#define BOARD_PIN_USB_PULLUP         GPIO3
#define BOARD_USB_HIGH_IS_PULLUP     true

/* Currently you can only use SPI1, since it has highest clock. */

#endif /* __BOARD_H__ */
