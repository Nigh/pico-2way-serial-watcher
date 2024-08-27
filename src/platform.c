#include "platform.h"
#include "hardware/gpio.h"
#include "usb_func.h"

char log_cache[128] = { 0 };
char log_buffer[512] = { 0 };
uint16_t log_ptr = 0;
int16_t log_length = 0;
