#ifndef __STM32_VSERPOG_USB_CDC_H__
#define __STM32_VSERPOG_USB_CDC_H__

#include <stdint.h>
#include <stddef.h>

#define USBCDC_PKT_SIZE_DAT 64
#define USBCDC_PKT_SIZE_INT 16

extern char usbcdc_rxbuf[USBCDC_PKT_SIZE_DAT]; /* DMA needs access */
extern volatile bool usb_ready;

void     usbcdc_init(void);
uint16_t usbcdc_write(void *buf, size_t len);
uint16_t usbcdc_putc(char c);
uint16_t usbcdc_putu32(uint32_t word);
uint16_t usbcdc_fetch_packet(void);
char     usbcdc_getc(void);
uint32_t usbcdc_getu24(void);
uint32_t usbcdc_getu32(void);
uint8_t  usbcdc_get_remainder(char **bufpp);

#endif /* __STM32_VSERPOG_USB_CDC_H__ */
