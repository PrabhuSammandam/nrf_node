/*
 * console.h
 *
 * Created: 20-03-2016 17:12:59
 *  Author: prabhu
 */ 


#ifndef CONSOLE_H_
#define CONSOLE_H_

void console_print_byte(uint8_t data);
void console_print_short(uint16_t data);
void console_print_long(uint32_t data);

/* Prints the given number in specified base */
void console_print_number(uint32_t number, uint8_t base);

#endif /* CONSOLE_H_ */