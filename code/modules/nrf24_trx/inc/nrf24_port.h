#ifndef __NRF24_HAL_H__
#define __NRF24_HAL_H__

#include <stdio.h>
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include "ioport/io_port.h"
#include "gpio/gpio.h"

#define NRF24_DEBUG_PRINTF(x) \
    do                        \
    {                         \
        x;                    \
    } while(0);

#define CSN_PIN IOPORT_CREATE_PIN(PORT_B, 1)
#define CE_PIN  IOPORT_CREATE_PIN(PORT_C, 0)

#define CSN_HIGH()             \
    do                         \
    {                          \
        nrf24_port_csn_high(); \
    } while(0);
#define CSN_LOW()             \
    do                        \
    {                         \
        nrf24_port_csn_low(); \
    } while(0);

#define CE_HIGH()             \
    do                        \
    {                         \
        nrf24_port_ce_high(); \
    } while(0);
#define CE_LOW()             \
    do                       \
    {                        \
        nrf24_port_ce_low(); \
    } while(0);

void        nrf24_port_init(void);

void        nrf24_port_delay_us(uint16_t delay_us);
void        nrf24_port_delay_ms(uint16_t delay_ms);

void        nrf24_port_csn_high(void);
void        nrf24_port_csn_low(void);

void        nrf24_port_ce_high(void);
void        nrf24_port_ce_low(void);

uint32_t    nrf24_port_current_time_ms(void);

uint8_t     nrf24_port_spi_write_byte(uint8_t data);

void     nrf24_port_radio_isr_function();

#endif /*__NRF24_HAL_H__*/
