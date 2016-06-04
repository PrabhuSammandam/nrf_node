/*
 * console.c
 *
 * Created: 20-03-2016 17:13:50
 *  Author: prabhu
 */ 
 #include <avr/io.h>
 #include "console.h"
 #include "serial/uart.h"

void console_print_byte(uint8_t data)
{
	console_print_number((uint32_t)data, 10);
}

void console_print_short(uint16_t data)
{
    console_print_number((uint32_t)data, 10);
}

void console_print_long(uint32_t data)
{
    console_print_number((uint32_t)data, 10);
}

void console_print_number(uint32_t number, uint8_t base)
{
  char buf[8 * sizeof(uint32_t) + 1]; // Assumes 8-bit chars plus null char.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do {
	  char c = number % base;
	  number /= base;

	  *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(number);

  uart_puts(str);
}

