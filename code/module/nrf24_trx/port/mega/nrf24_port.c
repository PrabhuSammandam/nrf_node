#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include "io_port.h"
#include "spi.h"
#include "tick_timer.h"

#include "nrf24_port.h"

#define CHIP_SELECT IOPORT_CREATE_PIN(PORT_C, 1)
#define CHIP_ENABLE IOPORT_CREATE_PIN(PORT_C, 0)

void nrf24_port_init(void)
{
    io_port_set_pin_output(CHIP_SELECT);
    io_port_set_pin_output(CHIP_ENABLE);
}

void nrf24_port_delay_us(uint16_t    delay_us)
{
    timer_delay_us(delay_us);
}

void nrf24_port_delay_ms(uint16_t    delay_ms)
{
    timer_delay_ms(delay_ms);
}

void nrf24_port_set_chip_select(uint8_t pin_value)
{
    if(pin_value) {
        io_port_set_pin_high(CHIP_SELECT);
    }
    else {
        io_port_set_pin_low(CHIP_SELECT);
    }
}

void nrf24_port_set_chip_enable(uint8_t pin_value)
{
    if(pin_value) {
        io_port_set_pin_high(CHIP_ENABLE);
    }
    else {
        io_port_set_pin_low(CHIP_ENABLE);
    }
}

uint8_t nrf24_port_spi_write_byte(uint8_t data)
{
    return spi_write_byte(data);
}

uint32_t nrf24_port_current_time_ms(void)
{
    return timer_millis();
}
