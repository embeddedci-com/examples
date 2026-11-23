/* NUCLEO-H753ZI board support: clocks, console, LEDs, USB pins, ROM bootloader. */
#ifndef BOARD_H
#define BOARD_H
#include <stdbool.h>
#include <stdint.h>

void board_init(void);
uint32_t millis(void);
void delay_ms(uint32_t ms);

void con_write(const char *s);
void con_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
int con_getc(void);

void led_green(bool on);   /* LD1: the host configured the device */
void led_yellow(bool on);  /* LD2: USB traffic */
void led_red(bool on);     /* LD3: error */
bool button_pressed(void); /* B1 */
bool vbus_present(void);   /* PA9, VBUS on CN13 */
void crs_state(unsigned *trim, bool *synced); /* HSI48 trim, and whether the CRS locked on the host's SOF */

void usb_hw_init(void);
void usb_hw_deinit(void);
void board_reset(void) __attribute__((noreturn));
void board_enter_rom_dfu(void) __attribute__((noreturn));

#endif
