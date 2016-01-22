#ifndef __STM32_VSERPOG_SPI_H__
#define __STM32_VSERPOG_SPI_H__

#include <stdint.h>
#include <stddef.h>

#include <libopencm3/stm32/gpio.h>

#define SPI_DEFAULT_CLOCK 10000000

#define SPI_SELECT()   gpio_clear(GPIOA, GPIO_SPI1_NSS)
#define SPI_UNSELECT() gpio_set(GPIOA, GPIO_SPI1_NSS)

uint32_t spi_setup(uint32_t speed_hz);
void spi_bulk_read(uint32_t rlen);
void spi_bulk_write(uint32_t slen);

#endif /* __STM32_VSERPOG_SPI_H__ */
