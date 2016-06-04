/*
 * io_port.c
 *
 * Created: 20-03-2016 08:27:10
 *  Author: prabhu
 */ 

 #include "io_port.h"

 const uint16_t PROGMEM port_to_mode_table[] = {0, (uint16_t)&DDRB,(uint16_t)&DDRC,(uint16_t)&DDRD};
 const uint16_t PROGMEM port_to_output_table[] = {0, (uint16_t)&PORTB,(uint16_t)&PORTC,(uint16_t)&PORTD};
 const uint16_t PROGMEM port_to_input_table[] = {0, (uint16_t)&PINB,(uint16_t)&PINC,(uint16_t)&PIND};
