#include <stdbool.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#ifdef STM32F0
#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/syscfg.h>
#endif /* STM32F0 */

#include "flashrom/serprog.h"
#include "flashrom/flash.h" /* For bus type */

#include "board.h"
#include "usbcdc.h"
#include "spi.h"

#define S_IFACE_VERSION   0x01             /* Currently version 1 */
#define S_PGM_NAME        "stm32-vserprog" /* The program's name, must < 16 bytes */
#define S_SUPPORTED_BUS   BUS_SPI
#define S_CMD_MAP ( \
  (1 << S_CMD_NOP)       | \
  (1 << S_CMD_Q_IFACE)   | \
  (1 << S_CMD_Q_CMDMAP)  | \
  (1 << S_CMD_Q_PGMNAME) | \
  (1 << S_CMD_Q_SERBUF)  | \
  (1 << S_CMD_Q_BUSTYPE) | \
  (1 << S_CMD_SYNCNOP)   | \
  (1 << S_CMD_O_SPIOP)   | \
  (1 << S_CMD_S_BUSTYPE) | \
  (1 << S_CMD_S_SPI_FREQ)| \
  (1 << S_CMD_S_PIN_STATE) \
)

#ifdef STM32F0
#define LED_ENABLE()  { \
  gpio_mode_setup(BOARD_PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BOARD_PIN_LED); \
  gpio_set_output_options(BOARD_PORT_LED, GPIO_OTYPE_PP, GPIO_OSPEED_LOW, BOARD_PIN_LED); \
}
#define LED_DISABLE() gpio_mode_setup(BOARD_PORT_LED, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_LED);
#else
#define LED_ENABLE()  gpio_set_mode(BOARD_PORT_LED, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, BOARD_PIN_LED)
#define LED_DISABLE() gpio_set_mode(BOARD_PORT_LED, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, BOARD_PIN_LED)
#endif /* STM32F0 */

#if BOARD_LED_HIGH_IS_BUSY
#define LED_BUSY()    gpio_set(BOARD_PORT_LED, BOARD_PIN_LED)
#define LED_IDLE()    gpio_clear(BOARD_PORT_LED, BOARD_PIN_LED)
#else
#define LED_BUSY()    gpio_clear(BOARD_PORT_LED, BOARD_PIN_LED)
#define LED_IDLE()    gpio_set(BOARD_PORT_LED, BOARD_PIN_LED)
#endif /* BOARD_LED_HIGH_IS_BUSY */

void handle_command(unsigned char command) {
  static uint8_t   i;        /* Loop                */
  static uint8_t   l;        /* Length              */
  static uint32_t  slen;     /* SPIOP write length  */
  static uint32_t  rlen;     /* SPIOP read length   */
  static uint32_t  freq_req; /* Requested SPI clock */

  LED_BUSY();

  switch(command) {
    case S_CMD_NOP: {
      usbcdc_putc(S_ACK);
      break;
    }

    case S_CMD_Q_IFACE: {
      // TODO: use usbcdc_write for better efficiency
      usbcdc_putc(S_ACK);

      /* little endian multibyte value to complete to 16bit */
      usbcdc_putc(S_IFACE_VERSION);
      usbcdc_putc(0);

      break;
    }

    case S_CMD_Q_CMDMAP: {
      // TODO: use usbcdc_write for better efficiency
      usbcdc_putc(S_ACK);

      /* little endian */
      usbcdc_putu32(S_CMD_MAP);

      for(i = 0; i < 32 - sizeof(uint32_t); i++) {
        usbcdc_putc(0);
      }

      break;
    }

    case S_CMD_Q_PGMNAME: {
      // TODO: use usbcdc_write for better efficiency
      usbcdc_putc(S_ACK);

      l = 0;
      while(S_PGM_NAME[l]) {
        usbcdc_putc(S_PGM_NAME[l]);
        l ++;
      }

      for(i = l; i < 16; i++) {
        usbcdc_putc(0);
      }

      break;
    }

    case S_CMD_Q_SERBUF: {
      // TODO: use usbcdc_write for better efficiency
      usbcdc_putc(S_ACK);

      /* Pretend to be 64K (0xffff) */
      usbcdc_putc(0xff);
      usbcdc_putc(0xff);

      break;
    }

    case S_CMD_Q_BUSTYPE: {
      // TODO: use usbcdc_write for better efficiency
      // TODO: LPC / FWH IO support via PP-Mode
      usbcdc_putc(S_ACK);
      usbcdc_putc(S_SUPPORTED_BUS);

      break;
    }

    case S_CMD_Q_CHIPSIZE: {
      break;
    }

    case S_CMD_Q_OPBUF: {
      // TODO: opbuf function 0
      break;
    }

    case S_CMD_Q_WRNMAXLEN: {
      break;
    }

    case S_CMD_R_BYTE: {
      break;
    }

    case S_CMD_R_NBYTES: {
      break;
    }

    case S_CMD_O_INIT: {
      break;
    }

    case S_CMD_O_WRITEB: {
      // TODO: opbuf function 1
      break;
    }

    case S_CMD_O_WRITEN: {
      // TODO: opbuf function 2
      break;
    }

    case S_CMD_O_DELAY: {
      // TODO: opbuf function 3
      break;
    }

    case S_CMD_O_EXEC: {
      // TODO: opbuf function 4
      break;
    }

    case S_CMD_SYNCNOP: {
      // TODO: use usbcdc_write for better efficiency
      usbcdc_putc(S_NAK);
      usbcdc_putc(S_ACK);

      break;
    }

    case S_CMD_Q_RDNMAXLEN: {
      // TODO
      break;
    }

    case S_CMD_S_BUSTYPE: {
      /* We do not have multiplexed bus interfaces,
       * so simply ack on supported types, no setup needed. */
      if((usbcdc_getc() | S_SUPPORTED_BUS) == S_SUPPORTED_BUS) {
        usbcdc_putc(S_ACK);
      } else {
        usbcdc_putc(S_NAK);
      }

      break;
    }

    case S_CMD_O_SPIOP: {
      slen = usbcdc_getu24();
      rlen = usbcdc_getu24();

      SPI_SELECT();

      /* TODO: handle errors with S_NAK */
      if(slen) {
        spi_bulk_write(slen);
      }
      usbcdc_putc(S_ACK); // TODO: S_ACK early for better performance (so while DMA is working, programmer can receive next command)?
      if(rlen) {
        spi_bulk_read(rlen); // TODO: buffer?
      }

      SPI_UNSELECT();
      break;
    }

    case S_CMD_S_SPI_FREQ: {
      freq_req = usbcdc_getu32();

      if(freq_req == 0) {
        usbcdc_putc(S_NAK);
      } else {
        usbcdc_putc(S_ACK);
        usbcdc_putu32(spi_setup(freq_req));
      }

      break;
    }

    case S_CMD_S_PIN_STATE: {
      if( usbcdc_getc() )
          spi_enable_pins();
      else
          spi_disable_pins();
      usbcdc_putc(S_ACK);
      break;
    }

    default: {
      break; // TODO: notify protocol failure malformed command
    }
  }

  LED_IDLE();
}

#ifdef GD32F103
#define  RCC_GCFGR_ADCPS_BIT_2  (1UL << 28)
#define  RCC_GCFGR_USBPS_DIV2   (3UL << 22)
#define  RCC_GCFGR_PLLMF_BIT_4  (1UL << 27)
/* GD32 extra PLL bit flip turns MUL9 (0b00111) into MUL24 (0b10111) */
#define	 RCC_CFGR_PLLMUL_PLL_CLK_MUL24	RCC_CFGR_PLLMUL_PLL_CLK_MUL9

#ifdef GD32F103_8MHZ
static void rcc_clock_setup_in_hse_8mhz_out_96mhz(void) {
	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

	/* Enable external high-speed oscillator 8MHz. */
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

  /*
	 * Set prescalers for AHB, ADC, ABP1, ABP2.
	 * Do this before touching the PLL. For reasons.
	 */
	rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);    /* Set. 96MHz Max. 108MHz */
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV8);  /* Set. 12MHz Max. 14MHz */
	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);     /* Set. 48MHz Max. 54MHz */
	rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);    /* Set. 96MHz Max. 108MHz */
  RCC_CFGR |= RCC_GCFGR_USBPS_DIV2;            /* USB Set. 48MHz  Max. 48MHz  */


  /* GD32 has 0-wait-state flash, do not touch anything! */

	/*
	 * Set the PLL multiplication factor to 24
	 * 8MHz (external) * 24 (multiplier) / 2 (pllprediv) = 96MHz
	 */
  /* GD32 extra PLL bit flip turns MUL9 (0b00111) into MUL24 (0b10111) */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL24);
  RCC_CFGR |= RCC_GCFGR_PLLMF_BIT_4;

	/* Select HSE as PLL source. */
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

	/*
	 * External frequency divided/2 before entering PLL
	 * (only valid/needed for HSE on GD32. Don't ask me why...).
	 */
	rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK_DIV2);

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

	/* Set the peripheral clock frequencies used */
	rcc_ahb_frequency = 96000000;
	rcc_apb1_frequency = 48000000;
	rcc_apb2_frequency = 96000000;
}
#else
static void rcc_clock_setup_in_hse_12mhz_out_96mhz(void) {
  /* Enable internal high-speed oscillator. */
  rcc_osc_on(RCC_HSI);
  rcc_wait_for_osc_ready(RCC_HSI);

  /* Select HSI as SYSCLK source. */
  rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

  /* Enable external high-speed oscillator 12MHz. */
  rcc_osc_on(RCC_HSE);
  rcc_wait_for_osc_ready(RCC_HSE);
  rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

  /*
   * Set prescalers for AHB, ADC, ABP1, ABP2.
   * Do this before touching the PLL
   */
  rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);   /* Set. 96MHz Max. 108MHz */
  rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV8); /* Set. 12MHz Max. 14MHz  */
  rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);    /* Set. 48MHz Max. 54MHz  */
  rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);   /* Set. 96MHz Max. 108MHz */
  RCC_CFGR |= RCC_GCFGR_USBPS_DIV2;           /* USB Set. 48MHz  Max. 48MHz  */

  /* GD32 has 0-wait-state flash, do not touch anything! */

  /*
   * Set the PLL multiplication factor to 8.
   * 12MHz (external) / 2 (prescale) * 16 (multiplier) = 96MHz
   */
  rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL16);

  /* Select HSE as PLL source. */
  rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

	/*
	 * External frequency divided/2 before entering PLL
	 * (only valid/needed for HSE on GD32. Don't ask me why...).
	 */
  rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK_DIV2);

  /* Enable PLL oscillator and wait for it to stabilize. */
  rcc_osc_on(RCC_PLL);
  rcc_wait_for_osc_ready(RCC_PLL);

  /* Select PLL as SYSCLK source. */
  rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

  /* Set the peripheral clock frequencies used */
  rcc_ahb_frequency  = 96000000;
  rcc_apb1_frequency = 48000000;
  rcc_apb2_frequency = 96000000;
}
#endif /* GD32F103_8MHZ */
#endif /* GD32F103 */

int main(void) {
  uint32_t i;

  rcc_periph_clock_enable(BOARD_RCC_LED);
  LED_ENABLE();
  LED_BUSY();

/* Setup clock accordingly */
#ifdef GD32F103
#ifdef GD32F103_8MHZ
  rcc_clock_setup_in_hse_8mhz_out_96mhz();
#else
  rcc_clock_setup_in_hse_12mhz_out_96mhz();
#endif /* GD32F103_8MHZ */
#else
#ifdef STM32F0
  rcc_clock_setup_in_hsi48_out_48mhz();
  rcc_periph_clock_enable(RCC_SYSCFG_COMP);
  SYSCFG_CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
  rcc_periph_clock_enable(RCC_CRS);
  crs_autotrim_usb_enable();
  rcc_set_usbclk_source(RCC_HSI48);
#else
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
#endif /* STM32F0 */
#endif /* GD32F103 */

  rcc_periph_clock_enable(RCC_GPIOA); /* For USB */

/* STM32F0x2 has internal pullup and does not need AFIO */
#ifndef STM32F0
  rcc_periph_clock_enable(BOARD_RCC_USB_PULLUP);
  rcc_periph_clock_enable(RCC_AFIO); /* For SPI */
#endif /* STM32F0 */

#if BOARD_USE_DEBUG_PINS_AS_GPIO
  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, AFIO_MAPR_TIM2_REMAP_FULL_REMAP);
#endif

/* Setup GPIO to pull up the D+ high. (STM32F0x2 has internal pullup.) */
#ifndef STM32F0
  gpio_set_mode(BOARD_PORT_USB_PULLUP, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, BOARD_PIN_USB_PULLUP);
#if BOARD_USB_HIGH_IS_PULLUP
  gpio_set(BOARD_PORT_USB_PULLUP, BOARD_PIN_USB_PULLUP);
#else
  gpio_clear(BOARD_PORT_USB_PULLUP, BOARD_PIN_USB_PULLUP);
#endif /* BOARD_USB_HIGH_IS_PULLUP */
#endif /* STM32F0 */

  usbcdc_init();
#ifdef HAS_BOARD_INIT
  board_init();
#endif

  spi_setup(SPI_DEFAULT_CLOCK);

  /* The loop. */
  while (true) {
    /* Wait and blink if USB is not ready. */
    LED_IDLE();
    while (!usb_ready) {
      LED_DISABLE();
      for (i = 0; i < rcc_ahb_frequency / 150; i ++) {
        asm("nop");
      }
      LED_ENABLE();
      for (i = 0; i < rcc_ahb_frequency / 150; i ++) {
        asm("nop");
      }
    }

    /* Actual thing */
    /* TODO: we are blocked here, hence no knowledge about USB bet reset. */
    handle_command(usbcdc_getc());
  }

  return 0;
}

/* Interrupts (currently none here) */

static void signal_fault(void) {
  uint32_t i;

  while (true) {
    LED_ENABLE();
    LED_BUSY();
    for (i = 0; i < 5000000; i ++) {
      asm("nop");
    }
    LED_DISABLE();
    for (i = 0; i < 5000000; i ++) {
      asm("nop");
    }
  }
}

void nmi_handler(void)
__attribute__ ((alias ("signal_fault")));

void hard_fault_handler(void)
__attribute__ ((alias ("signal_fault")));
