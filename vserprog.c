#include <stdbool.h>
#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>

#include "flashrom/serprog.h"
#include "flashrom/flash.h" /* For bus type */

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
  (1 << S_CMD_S_SPI_FREQ)  \
)

#define LED_ENABLE()  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0)
#define LED_DISABLE() gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0)
#define LED_BUSY()    gpio_set(GPIOA, GPIO0)
#define LED_IDLE()    gpio_clear(GPIOA, GPIO0)

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
      // TODO: OE
      break;
    }

    default: {
      break; // TODO: notify protocol failure malformed command
    }
  }

  LED_IDLE();
}

int main(void) {
  uint32_t i;

  rcc_periph_clock_enable(RCC_GPIOA);
  LED_ENABLE();
  LED_BUSY();

  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_AFIO);
  gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF, AFIO_MAPR_TIM2_REMAP_FULL_REMAP);

  /* Setup PB3 to pull up the D+ high. */
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO3);
  gpio_set(GPIOB, GPIO3);

  usbcdc_init();
  spi_setup(SPI_DEFAULT_CLOCK);
  /* Wait 500ms for USB setup to complete before trying to send anything. */
  /* FIXME: in ST's USB library there is some way to tell whether USB has finished initialization. */
  for (i = 0; i < 10000000; i ++) {
    asm("nop");
  }
  LED_IDLE();

  /* The loop. */
  while (true) {
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
