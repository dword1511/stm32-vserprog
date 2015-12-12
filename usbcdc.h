#ifndef __STM32_VSERPOG_USB_CDC_H__
#define __STM32_VSERPOG_USB_CDC_H__

#include <stdint.h>
#include <stddef.h>

#define USBCDC_PKT_SIZE_DAT 64
#define USBCDC_PKT_SIZE_INT 16

void     usbcdc_init(void);
uint16_t usbcdc_write(void *buf, size_t len);
uint16_t usbcdc_putc(char c);
uint16_t usbcdc_putu32(uint32_t word);
char     usbcdc_getc(void);
uint32_t usbcdc_getu24(void);
uint32_t usbcdc_getu32(void);

#endif /* __STM32_VSERPOG_USB_CDC_H__ */
