/*
* spi.c
*
* Created: 20-03-2016 19:00:17
*  Author: prabhu
*/
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include "ioport/io_port.h"
#include "spi.h"

#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define SPI_PIN_SS      IOPORT_CREATE_PIN(PORT_B, 2)
#define SPI_PIN_MOSI    IOPORT_CREATE_PIN(PORT_B, 3)
#define SPI_PIN_MISO    IOPORT_CREATE_PIN(PORT_B, 4)
#define SPI_PIN_SCK     IOPORT_CREATE_PIN(PORT_B, 5)
#endif

void spi_init(void)
{
    io_port_set_pin_output(SPI_PIN_MOSI);
    io_port_set_pin_output(SPI_PIN_SCK);
    io_port_set_pin_input(SPI_PIN_MISO);

    // if the SS pin is not already configured as an output
    // then set it high (to enable the internal pull-up resistor)
    if(io_port_is_pin_input(SPI_PIN_SS)) {
        io_port_set_pin_high(SPI_PIN_SS);
    }

    // When the SS pin is set as OUTPUT, it can be used as
    // a general purpose output port (it doesn't influence
    // SPI operations).
    io_port_set_pin_output(SPI_PIN_SS);

    // Warning: if the SS pin ever becomes a LOW INPUT then SPI
    // automatically switches to Slave, so the data direction of
    // the SS pin MUST be kept as OUTPUT.
    SPCR |= _BV(MSTR);  // Master mode
    SPCR &= ~_BV(DORD); // MSB first

    //SPSR |= _BV(SPI2X);
    //SPCR |= _BV(SPR0);

    // enable spi
    SPCR |= _BV(SPE);
}
