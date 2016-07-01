#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include "ioport/io_port.h"
#include "spi/spi.h"
#include "tick_timer/tick_timer.h"

#include "nrf24_port.h"

void nrf24_port_init(void)
{
    io_port_set_pin_output(CSN_PIN);
    io_port_set_pin_output(CE_PIN);

    /* enable the external interrupt 0 */
    EIMSK |= _BV(INT0);
}

void nrf24_port_delay_us(uint16_t    delay_us)
{
    timer_delay_us(delay_us);
}

void nrf24_port_delay_ms(uint16_t    delay_ms)
{
    timer_delay_ms(delay_ms);
}

uint8_t nrf24_port_spi_write_byte(uint8_t data)
{
    return spi_write_byte(data);
}

uint32_t nrf24_port_current_time_ms(void)
{
    return timer_millis();
}

void nrf24_port_csn_high(void)
{
    gpio_set_pin_high(CSN_PIN);
}

void nrf24_port_csn_low(void)
{
    gpio_set_pin_low(CSN_PIN);
}

void nrf24_port_ce_high(void)
{
    gpio_set_pin_high(CE_PIN)
}

void nrf24_port_ce_low(void)
{
    gpio_set_pin_low(CE_PIN)
}

ISR(INT0_vect)
{
    nrf24_port_radio_isr_function();
}
