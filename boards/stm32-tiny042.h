#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/*
 * Board definitions for STM32 Tiny042
 * This is my spare-time project. It is a minimalist USB board for STM32F0x2.
 * OSHPark:
 *      https://oshpark.com/shared_projects/rBnfauMK
 * GitHub:
 *      https://github.com/dword1511/stm32-tiny042.git
 */

#define BOARD_USE_DEBUG_PINS_AS_GPIO false

#define BOARD_RCC_LED                RCC_GPIOA
#define BOARD_PORT_LED               GPIOA
#define BOARD_PIN_LED                GPIO14
#define BOARD_LED_HIGH_IS_BUSY       false /* Only LED, high active, use as idle. */

/* STM32F0x2 has internal USB pullup. */

/* Currently you can only use SPI1, since it has highest clock. */

#endif /* __BOARD_H__ */
