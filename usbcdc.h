#ifndef __STM32_FREQMETER_USB_CDC_H__
#define __STM32_FREQMETER_USB_CDC_H__

#include <stdint.h>
#include <stddef.h>

void usbcdc_init(void);
uint16_t usbcdc_write(char* buf, size_t len);
char usbcdc_getc(void);

#endif /* __STM32_FREQMETER_USB_CDC_H__ */
