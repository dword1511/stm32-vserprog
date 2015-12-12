#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "spi.h"

uint32_t spi_setup(uint32_t speed_hz) {
  uint32_t clkdiv;
  uint32_t relspd;

  /* SPI bus is on APB2 which runs at 72MHz. */
  /* Lowest available */
  clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_256;
  relspd = 281250;

  if(speed_hz >= 562500) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_128;
    relspd = 562500;
  }

  if(speed_hz >= 1125000) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_64;
    relspd = 1125000;
  }

  if(speed_hz >= 2250000) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_32;
    relspd = 2250000;
  }

  if(speed_hz >= 4500000) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_16;
    relspd = 4500000;
  }

  if(speed_hz >= 9000000) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_8;
    relspd = 9000000;
  }

  if(speed_hz >= 18000000) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_4;
    relspd = 18000000;
  }

  if(speed_hz >= 36000000) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_2;
    relspd = 36000000;
  }

  /* Configure GPIOs: SS = PA4, SCK = PA5, MISO = PA6, MOSI = PA7 */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO7);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4); /* SS is manual */

  /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
  spi_reset(SPI1);

  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/256 of peripheral clock frequency
   * Clock polarity: Idle Low
   * Clock phase: Data valid on rising edge (1st edge for idle low)
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
  spi_init_master(SPI1, clkdiv, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * NOTE:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI1);
  spi_set_nss_high(SPI1);

  /* Misc. */
  spi_disable_crc(SPI1);
  spi_disable_error_interrupt(SPI1);
  spi_disable_rx_buffer_not_empty_interrupt(SPI1);
  spi_disable_tx_buffer_empty_interrupt(SPI1);
  spi_set_full_duplex_mode(SPI1);

  /* Enable SPI1 periph. */
  spi_enable(SPI1);

  /* Report actual clock speed selected */
  return relspd;
}

/* TODO: DMA RW */
void spi_bulk_read(uint32_t rlen) {
  
}

void spi_bulk_write(uint32_t slen) {
  
}

