#ifndef __STM32_VSERPOG_SPI_H__
#define __STM32_VSERPOG_SPI_H__

#include <stdint.h>
#include <stddef.h>

#include <libopencm3/stm32/gpio.h>

/* libopencm3 for STM32F0x2 lacks AF GPIO definitions. This is for AF0. */
#ifdef STM32F0
#define GPIO_SPI1_NSS  GPIO4 /* PA4 */
#define GPIO_SPI1_SCK  GPIO5 /* PA5 */
#define GPIO_SPI1_MISO GPIO6 /* PA6 */
#define GPIO_SPI1_MOSI GPIO7 /* PA7 */
#endif /* STM32F0 */
#define GPIO_BANK_SPI1 GPIOA

#define SPI_DEFAULT_CLOCK 10000000

#define SPI_SELECT()   gpio_clear(GPIOA, GPIO_SPI1_NSS)
#define SPI_UNSELECT() gpio_set(GPIOA, GPIO_SPI1_NSS)

uint32_t spi_setup(uint32_t speed_hz);
void spi_bulk_read(uint32_t rlen);
void spi_bulk_write(uint32_t slen);
void spi_enable_pins(void);
void spi_disable_pins(void);

#endif /* __STM32_VSERPOG_SPI_H__ */
