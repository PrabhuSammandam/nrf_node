/*
 * io_port.h
 *
 * Created: 20-03-2016 08:24:36
 *  Author: prabhu
 */ 


#ifndef IO_PORT_H_
#define IO_PORT_H_

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>
#include "compiler.h"

extern const uint16_t PROGMEM port_to_mode_table[];
extern const uint16_t PROGMEM port_to_output_table[];
extern const uint16_t PROGMEM port_to_input_table[];

#define io_port_id_to_mode_base(p) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_table + (p))) )
#define io_port_id_to_output_base(p) ( (volatile uint8_t *)( pgm_read_word( port_to_output_table + (p))) )
#define io_port_id_to_input_base(p) ( (volatile uint8_t *)( pgm_read_word( port_to_input_table + (p))) )

#define IOPORT_PORT_B    1
#define IOPORT_PORT_C    2
#define IOPORT_PORT_D    3

#define IO_PORT_PIN_MASK(P) (0x1 << (P & 0x07))

#define IOPORT_CREATE_PIN(port, pin) ((IOPORT_ ## port) * 8 + (pin))

__always_inline void io_port_set_pin_input(uint8_t pin)
{
	volatile uint8_t* port = io_port_id_to_mode_base(pin >> 3);

	*port &= ~IO_PORT_PIN_MASK(pin);
}

__always_inline void io_port_set_pin_output(uint8_t pin)
{
	volatile uint8_t* port_mode_reg = io_port_id_to_mode_base(pin >> 3);

	*port_mode_reg |= IO_PORT_PIN_MASK(pin);
}

__always_inline uint8_t io_port_is_pin_output(uint8_t pin)
{
	volatile uint8_t* port_mode_reg = io_port_id_to_mode_base(pin >> 3);

	return ((*port_mode_reg & IO_PORT_PIN_MASK(pin)) != 0);
}

__always_inline uint8_t io_port_is_pin_input(uint8_t pin)
{
	volatile uint8_t* port_mode_reg = io_port_id_to_mode_base(pin >> 3);

	return ((*port_mode_reg & IO_PORT_PIN_MASK(pin)) == 0);
}

__always_inline void io_port_set_pin_high(uint8_t pin)
{
	volatile uint8_t* port_output_reg = io_port_id_to_output_base(pin >> 3);

	*port_output_reg |= IO_PORT_PIN_MASK(pin);
}

__always_inline void io_port_set_pin_low(uint8_t pin)
{
	volatile uint8_t* port_output_reg = io_port_id_to_output_base(pin >> 3);

	*port_output_reg &= ~IO_PORT_PIN_MASK(pin);
}

__always_inline void io_port_toggle_pin(uint8_t pin)
{
	volatile uint8_t* port_output_reg = io_port_id_to_output_base(pin >> 3);

	*port_output_reg ^= IO_PORT_PIN_MASK(pin);
}

__always_inline uint8_t io_port_is_pin_high(uint8_t pin)
{
	volatile uint8_t* port_input_reg = io_port_id_to_input_base(pin >> 3);

	return (((*port_input_reg) & IO_PORT_PIN_MASK(pin)) != 0);
}

__always_inline uint8_t io_port_is_pin_low(uint8_t pin)
{
	volatile uint8_t* port_input_reg = io_port_id_to_input_base(pin >> 3);

	return (((*port_input_reg) & IO_PORT_PIN_MASK(pin)) == 0);
}

#endif /* IO_PORT_H_ */