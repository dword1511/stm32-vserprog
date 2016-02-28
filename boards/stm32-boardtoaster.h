#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define BOARD_USE_DEBUG_PINS_AS_GPIO false

#define BOARD_RCC_LED                RCC_GPIOB
#define BOARD_PORT_LED               GPIOB
#define BOARD_PIN_LED                GPIO1
#define BOARD_LED_HIGH_IS_BUSY       true

/* STM32F0x2 has internal USB pullup. */

/* Currently you can only use SPI1, since it has highest clock. */

#endif /* __BOARD_H__ */
