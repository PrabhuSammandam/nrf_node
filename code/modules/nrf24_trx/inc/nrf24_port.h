#ifndef __NRF24_HAL_H__
#define __NRF24_HAL_H__

#include <stdio.h>

#define NRF24_DEBUG_PRINTF(x) do { x; } while (0);

#define NRF24_HAL_CHIP_SELECT_VALUE_LOW     0
#define NRF24_HAL_CHIP_SELECT_VALUE_HIGH    1

#define NRF24_HAL_CHIP_ENABLE_VALUE_LOW     0
#define NRF24_HAL_CHIP_ENABLE_VALUE_HIGH    1

void        nrf24_port_init(void);

void        nrf24_port_delay_us(uint16_t delay_us);
void        nrf24_port_delay_ms(uint16_t delay_ms);

uint32_t    nrf24_port_current_time_ms(void);

void        nrf24_port_set_chip_select(uint8_t pin_value);
void        nrf24_port_set_chip_enable(uint8_t pin_value);

uint8_t     nrf24_port_spi_write_byte(uint8_t data);

#endif /*__NRF24_HAL_H__*/
