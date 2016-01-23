#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>

#include "spi.h"
#include "usbcdc.h"

#define likely(x)   __builtin_expect((x), 1)
#define unlikely(x) __builtin_expect((x), 0)

/* DMA channel and requests has fixed mapping, do not change */
#define SPI_DMA_RX_CH DMA_CHANNEL2
#define SPI_DMA_TX_CH DMA_CHANNEL3

static uint8_t dma_rxbuf[USBCDC_PKT_SIZE_DAT];

uint32_t spi_setup(uint32_t speed_hz) {
  uint32_t clkdiv;
  uint32_t relspd;

  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable(RCC_DMA1);

  /* SPI1 is on APB2 which runs at 72MHz. Assume f = f_PCLK / 2 = 36MHz (whereas datasheet says 18MHz max but reference manual has no such word). */
  /* Lowest available */
  clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_256;
  relspd = rcc_apb2_frequency / 256;

  if(speed_hz >= rcc_apb2_frequency / 128) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_128;
    relspd = rcc_apb2_frequency / 128;
  }

  if(speed_hz >= rcc_apb2_frequency / 64) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_64;
    relspd = rcc_apb2_frequency / 64;
  }

  if(speed_hz >= rcc_apb2_frequency / 32) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_32;
    relspd = rcc_apb2_frequency / 32;
  }

  if(speed_hz >= rcc_apb2_frequency / 16) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_16;
    relspd = rcc_apb2_frequency / 16;
  }

  if(speed_hz >= rcc_apb2_frequency / 8) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_8;
    relspd = rcc_apb2_frequency / 8;
  }

  if(speed_hz >= rcc_apb2_frequency / 4) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_4;
    relspd = rcc_apb2_frequency / 4;
  }

  if(speed_hz >= rcc_apb2_frequency / 2) {
    clkdiv = SPI_CR1_BAUDRATE_FPCLK_DIV_2;
    relspd = rcc_apb2_frequency / 2;
  }

  /* Configure GPIOs: SS = PA4, SCK = PA5, MISO = PA6, MOSI = PA7 */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI1_SCK | GPIO_SPI1_MOSI);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_SPI1_MISO);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_SPI1_NSS); /* SS is manual */
  gpio_set(GPIOA, GPIO_SPI1_MISO);
  gpio_set(GPIOA, GPIO_SPI1_NSS);

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

static void spi_dma_write(uint16_t len, char *buf) {
  static uint8_t tmp = 0;

  /* Reset DMA channels */
  dma_channel_reset(DMA1, SPI_DMA_RX_CH);
  dma_channel_reset(DMA1, SPI_DMA_TX_CH);

  /* Configure TX */
  dma_set_peripheral_address(DMA1, SPI_DMA_TX_CH, (uint32_t)&SPI1_DR);
  dma_set_memory_address(DMA1, SPI_DMA_TX_CH, (uint32_t)buf);
  dma_set_number_of_data(DMA1, SPI_DMA_TX_CH, len);
  dma_set_read_from_memory(DMA1, SPI_DMA_TX_CH);
  dma_enable_memory_increment_mode(DMA1, SPI_DMA_TX_CH);
  dma_set_peripheral_size(DMA1, SPI_DMA_TX_CH, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, SPI_DMA_TX_CH, DMA_CCR_MSIZE_8BIT);
  dma_set_priority(DMA1, SPI_DMA_TX_CH, DMA_CCR_PL_HIGH);

  /* Configure RX (for collecting garbage from SPI controller) */
  dma_set_peripheral_address(DMA1, SPI_DMA_RX_CH, (uint32_t)&SPI1_DR);
  dma_set_memory_address(DMA1, SPI_DMA_RX_CH, (uint32_t)(&tmp));
  dma_set_number_of_data(DMA1, SPI_DMA_RX_CH, len);
  dma_set_read_from_peripheral(DMA1, SPI_DMA_RX_CH);
  dma_disable_memory_increment_mode(DMA1, SPI_DMA_RX_CH); /* Do not polute cache */
  dma_set_peripheral_size(DMA1, SPI_DMA_RX_CH, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, SPI_DMA_RX_CH, DMA_CCR_MSIZE_8BIT);
  dma_set_priority(DMA1, SPI_DMA_RX_CH, DMA_CCR_PL_VERY_HIGH);

  dma_enable_channel(DMA1, SPI_DMA_RX_CH);
  dma_enable_channel(DMA1, SPI_DMA_TX_CH);
  spi_enable_rx_dma(SPI1);
  spi_enable_tx_dma(SPI1);
}

static void spi_dma_read(uint16_t len) {
  static uint8_t tmp = 0;

  /* Reset DMA channels */
  dma_channel_reset(DMA1, SPI_DMA_RX_CH);
  dma_channel_reset(DMA1, SPI_DMA_TX_CH);

  /* Configure RX */
  dma_set_peripheral_address(DMA1, SPI_DMA_RX_CH, (uint32_t)&SPI1_DR);
  dma_set_memory_address(DMA1, SPI_DMA_RX_CH, (uint32_t)dma_rxbuf);
  dma_set_number_of_data(DMA1, SPI_DMA_RX_CH, len);
  dma_set_read_from_peripheral(DMA1, SPI_DMA_RX_CH);
  dma_enable_memory_increment_mode(DMA1, SPI_DMA_RX_CH);
  dma_set_peripheral_size(DMA1, SPI_DMA_RX_CH, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, SPI_DMA_RX_CH, DMA_CCR_MSIZE_8BIT);
  dma_set_priority(DMA1, SPI_DMA_RX_CH, DMA_CCR_PL_VERY_HIGH);

  /* Configure TX (for SPI clock) */
  dma_set_peripheral_address(DMA1, SPI_DMA_TX_CH, (uint32_t)&SPI1_DR);
  dma_set_memory_address(DMA1, SPI_DMA_TX_CH, (uint32_t)(&tmp));
  dma_set_number_of_data(DMA1, SPI_DMA_TX_CH, len);
  dma_set_read_from_memory(DMA1, SPI_DMA_TX_CH);
  dma_disable_memory_increment_mode(DMA1, SPI_DMA_TX_CH); /* Do not polute cache */
  dma_set_peripheral_size(DMA1, SPI_DMA_TX_CH, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, SPI_DMA_TX_CH, DMA_CCR_MSIZE_8BIT);
  dma_set_priority(DMA1, SPI_DMA_TX_CH, DMA_CCR_PL_HIGH);

  dma_enable_channel(DMA1, SPI_DMA_RX_CH);
  dma_enable_channel(DMA1, SPI_DMA_TX_CH);
  spi_enable_rx_dma(SPI1);
  spi_enable_tx_dma(SPI1);
}

/* Old CPU copying code */
static void spi_copy_to_usb(uint32_t len) {
  while (len) {
    spi_send(SPI1, 0x00);
    usbcdc_putc(spi_read(SPI1));
    len --;
  }
}

static void spi_copy_from_usb(uint32_t len) {
  while (len) {
    spi_send(SPI1, usbcdc_getc());
    spi_read(SPI1);
    len --;
  }
}

#ifdef GD32F103
/* FIXME: Currently SPI does not work on GD32 under any clock... */
void spi_bulk_read(uint32_t rlen) {
  spi_copy_to_usb(rlen);
}

void spi_bulk_write(uint32_t slen) {
  spi_copy_from_usb(slen);
}
#else
void spi_bulk_read(uint32_t rlen) {
  while (likely(rlen >= USBCDC_PKT_SIZE_DAT)) {
    spi_dma_read(USBCDC_PKT_SIZE_DAT);
    rlen -= USBCDC_PKT_SIZE_DAT;

    while (unlikely(!dma_get_interrupt_flag(DMA1, SPI_DMA_RX_CH, DMA_TCIF))); /* It is liklely, but we want to exit loop with low latency. */
    dma_clear_interrupt_flags(DMA1, SPI_DMA_RX_CH, DMA_TCIF);
    spi_disable_rx_dma(SPI1);
    spi_disable_tx_dma(SPI1);
    dma_disable_channel(DMA1, SPI_DMA_RX_CH);
    dma_disable_channel(DMA1, SPI_DMA_TX_CH);

    usbcdc_write(dma_rxbuf, USBCDC_PKT_SIZE_DAT);
  }

  /* Leftover only happens when reading individual registers. */
  if (unlikely(rlen > 0)) {
    spi_dma_read(rlen);

    while(unlikely(!dma_get_interrupt_flag(DMA1, SPI_DMA_RX_CH, DMA_TCIF)));
    dma_clear_interrupt_flags(DMA1, SPI_DMA_RX_CH, DMA_TCIF);
    spi_disable_rx_dma(SPI1);
    spi_disable_tx_dma(SPI1);
    dma_disable_channel(DMA1, SPI_DMA_RX_CH);
    dma_disable_channel(DMA1, SPI_DMA_TX_CH);

    usbcdc_write(dma_rxbuf, rlen);
  }
}

void spi_bulk_write(uint32_t slen) {
  uint8_t urlen;
  char *urbuf;

  urlen = usbcdc_get_remainder(&urbuf);

  /* Due to the characteristics of flashrom and serprog protocol, slen >= urlen. */
  if (urlen > 0) {
    spi_dma_write(urlen, urbuf);
    slen -= urlen;

    /* Always check RX flag to avoid leftovers in SPI_DR, which messes data up. */
    while (unlikely(!dma_get_interrupt_flag(DMA1, SPI_DMA_RX_CH, DMA_TCIF))); /* It is liklely, but we want to exit loop with low latency. */
    dma_clear_interrupt_flags(DMA1, SPI_DMA_RX_CH, DMA_TCIF);
    spi_disable_rx_dma(SPI1);
    spi_disable_tx_dma(SPI1);
    dma_disable_channel(DMA1, SPI_DMA_RX_CH);
    dma_disable_channel(DMA1, SPI_DMA_TX_CH);
  }

  /* We have no control over packet size here. */
  while (likely(slen > 0)) {
    urlen = usbcdc_fetch_packet();
    spi_dma_write(urlen, usbcdc_rxbuf);
    slen -= urlen;

    while (unlikely(!dma_get_interrupt_flag(DMA1, SPI_DMA_RX_CH, DMA_TCIF))); /* It is liklely, but we want to exit loop with low latency. */
    dma_clear_interrupt_flags(DMA1, SPI_DMA_RX_CH, DMA_TCIF);
    spi_disable_rx_dma(SPI1);
    spi_disable_tx_dma(SPI1);
    dma_disable_channel(DMA1, SPI_DMA_RX_CH);
    dma_disable_channel(DMA1, SPI_DMA_TX_CH);
  }

  /* Mark USB RX buffer as used. */
  usbcdc_get_remainder(&urbuf);
}
#endif /* GD32F103 */
