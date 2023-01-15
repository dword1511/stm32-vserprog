#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/*
 * Board definitions for "redpill" and "bluepill" boards
 * These are low cost boards that are quite wide spread, and might come in many brands.
 * Wiki:
 *      http://wiki.stm32duino.com/index.php?title=Blue_Pill
 *      http://wiki.stm32duino.com/index.php?title=Red_Pill
 */

#define BOARD_USE_DEBUG_PINS_AS_GPIO false /* We don't have to disable SWD to access GPIO's on these boards */

#define BOARD_RCC_LED                RCC_GPIOC
#define BOARD_PORT_LED               GPIOC
#define BOARD_PIN_LED                GPIO13
#define BOARD_LED_HIGH_IS_BUSY       true

/*
 * Pull USB D+ low on startup, to re-enumerate.
 * Bluepill board lacks dedicated pin for handling this pull-up.
 * https://stm32world.com/wiki/STM32_USB_Device_Renumeration
 */
#define BOARD_RCC_USB_PULLUP         RCC_GPIOA
#define BOARD_PORT_USB_PULLUP        GPIOA
#define BOARD_PIN_USB_PULLUP         GPIO12
#define BOARD_USB_HIGH_IS_PULLUP     false

/* Currently you can only use SPI1, since it has highest clock. */

#endif /* __BOARD_H__ */
