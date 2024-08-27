
#include "pico/stdlib.h"
#include "hardware/sync.h"

#include "scheduler/uevent.h"
#include "scheduler/scheduler.h"

#include "platform.h"
#include "led_drv.h"

#include "tusb_config.h"

#include "pico/sync.h"
#include "pico/float.h"
#include "pico/bootrom.h"

#define UART_BAUDRATE 921600

critical_section_t scheduler_lock;
static __inline void CRITICAL_REGION_INIT(void) {
	critical_section_init(&scheduler_lock);
}
static __inline void CRITICAL_REGION_ENTER(void) {
	critical_section_enter_blocking(&scheduler_lock);
}
static __inline void CRITICAL_REGION_EXIT(void) {
	critical_section_exit(&scheduler_lock);
}

bool timer_4hz_callback(struct repeating_timer* t) {
	LOG_RAW("At %lld us:\n", time_us_64());
	uevt_bc_e(UEVT_TIMER_4HZ);
	return true;
}

#define U32RGB(r, g, b) (((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b))

void led_blink_routine(void) {
	static uint8_t _tick = 0;
	_tick += 1;
	if(_tick & 0x1) {
		if(usb_mounted) {
			ws2812_setpixel(U32RGB(4, 14, 4));
		} else {
			ws2812_setpixel(U32RGB(20, 20, 2));
		}
	} else {
		ws2812_setpixel(U32RGB(0, 0, 0));
	}
}

void main_handler(uevt_t* evt) {
	switch(evt->evt_id) {
		case UEVT_TIMER_4HZ:
			led_blink_routine();
			break;
	}
}

void uevt_log(char* str) {
	LOG_RAW("%s\n", str);
}

int actual_baudrate = 0;

static char serial_fifo[16];
static uint8_t serial_wp = 0;
uint8_t serial_got(const char* str) {
	uint8_t len = strlen(str);
	for(uint8_t i = 1; i <= len; i++) {
		if(serial_fifo[serial_wp + (0x10 - i) & 0xF] != str[len - i]) {
			return 0;
		}
	}
	return 1;
}
void serial_receive(uint8_t const* buffer, uint16_t bufsize) {
	for(uint16_t i = 0; i < bufsize; i++) {
		if((*buffer == 0x0A) || (*buffer == 0x0D)) {
			if(serial_got("UPLOAD")) {
				ws2812_setpixel(U32RGB(20, 0, 20));
				reset_usb_boot(0, 0);
			}
			if(serial_got("BAUDRATE")) {
				LOG_RAW("baudrate: %d\n", actual_baudrate);
			}
		} else {
			serial_fifo[serial_wp++ & 0xF] = *buffer++;
		}
	}
}

#include "hardware/uart.h"
static void uarts_init(void) {
	uart_init(uart0, 9600);
	gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));
	actual_baudrate = uart_set_baudrate(uart0, UART_BAUDRATE);
	uart_set_hw_flow(uart0, false, false);
	uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
	uart_set_fifo_enabled(uart0, true);

	uart_init(uart1, 9600);
	gpio_set_function(5, UART_FUNCSEL_NUM(uart1, 5));
	uart_set_hw_flow(uart1, false, false);
	uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
	uart_set_fifo_enabled(uart1, true);
}

#include "hardware/xosc.h"
extern void cdc_task(void);
int main() {
	xosc_init();

	uarts_init();

	CRITICAL_REGION_INIT();
	app_sched_init();
	user_event_init();
	user_event_handler_regist(main_handler);

	ws2812_setup();
	struct repeating_timer timer;
	add_repeating_timer_us(249978ul, timer_4hz_callback, NULL, &timer);
	tusb_init();
	cdc_log_init();
	while(true) {
		app_sched_execute();
		tud_task();
		cdc_task();
		__wfi();
	}
}
