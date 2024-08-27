#ifndef _USB_FUNC_H_
#define _USB_FUNC_H_
#include <stdint.h>
#include <stdbool.h>

extern volatile bool usb_mounted;
void cdc_log_init(void);
void cdc_task(void);
void cdc_log_print(char* str);

#endif
