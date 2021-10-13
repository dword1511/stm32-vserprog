#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/*
 * Board definitions for STM32 Alternate042
 * It is a reused PCB left from the IRUSB project.
 * The board uses the same CPU as the stm32-tiny042 project, but in a 32 pin
 * package.
 * Instead of PORTA pins 0 to 7, PORTA pins 1 to 8 are on a pin header.
 * Instead of a LED on port A GPIO14, its on GPIO15. It will be on if a high
 * level is applied.
 * The hardware supports SPI frequencies from 187.5kHz to 24MHz in power-of-two
 * steps.
 *
 * For flashing, close jumper J4 (connects BOOT pin to Vcc) and call
 * dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D stm32-vserprog.bin
 *
 * Connection:
 * Flash Chip:     STM32:
 * CS              PORTA pin 4 - SPI1_NSS
 * SCK             PORTA pin 5 - SPI1_SCK
 * SO or DO        PORTA pin 6 - SPI1_MISO
 * SI or DI        PORTA pin 7 - SPI1_MOSI
 * HOLD#       Vcc=3.3V
 * WP#         For some reason the winbond chip only gave reliable results when
 *             connected to GND.
 *
 * Tested Flash chips:
 * 8MBit: Winbond W25Q80BV -> detected as W25Q80.V, works with 24MHz, even when
 *        the clock looks more like a sine on a breadboard.
 * 16MBit: Macronix MX25L1606E -> the signature fits to three different chips.
 *         Set to "MX25L1605A/MX25L1606E/MX25L1608E". Has bit erros at 24MHz.
 *         Reads reliable with the slower frequencies. I would use 1.5MHz.
 *         Reading speed:
 *         375kHz: 46s.
 *         750kHz: 24s
 *         1.5MHz: 14s
 *         3MHz:   12s
 *         Faster frequencies don't improve the reading speed.
 *
 * The board can be found at:
 *      https://github.com/Solartraveler/irusb
 */

#define BOARD_USE_DEBUG_PINS_AS_GPIO false

#define BOARD_RCC_LED                RCC_GPIOA
#define BOARD_PORT_LED               GPIOA
#define BOARD_PIN_LED                GPIO15
#define BOARD_LED_HIGH_IS_BUSY       false /* Only LED, high active, use as idle. */

/* STM32F0x2 has internal USB pullup. */

/* Currently you can only use SPI1. */

#endif /* __BOARD_H__ */
