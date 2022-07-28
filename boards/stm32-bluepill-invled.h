#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/*
 * Board popular on Taobao, e.g. https://item.taobao.com/item.htm?id=630403001847 ,
 * very similar to bluepill, but with an LED connected to PC13 with inversed polarity (active low).
 */

#define BOARD_USE_DEBUG_PINS_AS_GPIO false /* We don't have to disable SWD to access GPIO's on these boards */

#define BOARD_RCC_LED                RCC_GPIOC
#define BOARD_PORT_LED               GPIOC
#define BOARD_PIN_LED                GPIO13
#define BOARD_LED_HIGH_IS_BUSY       false

#define BOARD_RCC_USB_PULLUP         RCC_GPIOA
#define BOARD_PORT_USB_PULLUP        GPIOA
#define BOARD_PIN_USB_PULLUP         GPIO12
#define BOARD_USB_HIGH_IS_PULLUP     true

/* Currently you can only use SPI1, since it has highest clock. */

#endif /* __BOARD_H__ */
