/*
* nrf24_hal.h
*
* Created: 20-03-2016 20:25:07
*  Author: prabhu sammandam
*/
#ifndef NRF24_H_
#define NRF24_H_

#include <stdint.h>
#include "nrf24l01.h"

/*$off*/

#define NRF24_HAL_TRUE      1
#define NRF24_HAL_FALSE     0

/*bit mask*/
typedef enum
{
    NRF24_HAL_RF_OUTPUT_POWER_18DBM = 0,    /**< Output power set to -18dBm */
    NRF24_HAL_RF_OUTPUT_POWER_12DBM = 0x02,        /**< Output power set to -12dBm */
    NRF24_HAL_RF_OUTPUT_POWER_6DBM = 0x04,         /**< Output power set to -6dBm  */
    NRF24_HAL_RF_OUTPUT_POWER_0DBM = 0x06          /**< Output power set to 0dBm   */
} nrf24_hal_rf_output_power_t;

/*bit mask*/
typedef enum
{
    NRF24_HAL_DATA_RATE_1MBPS       = 0,
    NRF24_HAL_DATA_RATE_2MBPS = 0x08,
    NRF24_HAL_DATA_RATE_250KBPS = 0x20
} nrf24_hal_datarate_t;

typedef enum
{
    NRF24_HAL_CRC_DISABLED          = 0,
    NRF24_HAL_CRC_8,
    NRF24_HAL_CRC_16
} nrf24_hal_crc_length_t;

typedef enum
{
    NRF24_HAL_PIPE_0                = 0,    /**< Select pipe0 */
    NRF24_HAL_PIPE_1,                       /**< Select pipe1 */
    NRF24_HAL_PIPE_2,                       /**< Select pipe2 */
    NRF24_HAL_PIPE_3,                       /**< Select pipe3 */
    NRF24_HAL_PIPE_4,                       /**< Select pipe4 */
    NRF24_HAL_PIPE_5,                       /**< Select pipe5 */
    NRF24_HAL_PIPE_TX,
    NRF24_HAL_PIPE_ALL              = 0xFF  /**< Close or open all pipes*/
} nrf24_hal_pipe_t;

typedef enum
{
    NRF24_HAL_IRQ_MAX_RT            = 4,    /**< Max retries interrupt */
    NRF24_HAL_IRQ_TX_DS,                    /**< TX Data Sent interrupt */
    NRF24_HAL_IRQ_RX_DR                     /**< RX Data Ready interrupt */
} nrf24_hal_irq_source_t;

typedef enum
{
    NRF24_HAL_ARD_250_US            = 0,
    NRF24_HAL_ARD_500_US            = 1,
    NRF24_HAL_ARD_750_US            = 2,
    NRF24_HAL_ARD_1000_US           = 3,
    NRF24_HAL_ARD_1250_US           = 4,
    NRF24_HAL_ARD_1500_US           = 5,
    NRF24_HAL_ARD_1750_US           = 6,
    NRF24_HAL_ARD_2000_US           = 7,
    NRF24_HAL_ARD_2250_US           = 8,
    NRF24_HAL_ARD_2500_US           = 9,
    NRF24_HAL_ARD_2750_US           = 10,
    NRF24_HAL_ARD_3000_US           = 11,
    NRF24_HAL_ARD_3250_US           = 12,
    NRF24_HAL_ARD_3500_US           = 13,
    NRF24_HAL_ARD_3750_US           = 14,
    NRF24_HAL_ARD_4000_US           = 15
} nrf24_hal_ard_t;

typedef enum
{
    NRF24_HAL_OP_MODE_TX,
    NRF24_HAL_OP_MODE_RX
} nrf24_nrf_operation_mode_t;

/*$on*/
void nrf24_hal_chip_enable_low(void);
void nrf24_hal_chip_enable_high(void);
void nrf24_hal_chip_enable_pulse(void);

uint8_t nrf24_hal_flush_rx_fifo(void);
uint8_t nrf24_hal_flush_tx_fifo(void);
void nrf24_hal_reuse_tx_payload(void);
uint8_t nrf24_hal_read_rx_payload_width(void);
void nrf24_hal_init(void);
uint8_t nrf24_hal_start(void);
void nrf24_hal_set_tx_mode(void);
void nrf24_hal_set_rx_mode(void);
void nrf24_hal_set_operation_mode(nrf24_nrf_operation_mode_t op_mode);

void nrf24_hal_write_tx_payload(const uint8_t* tx_payload, uint8_t length);
void nrf24_hal_write_tx_payload_noack(const uint8_t* tx_payload, uint8_t length);
void nrf24_hal_write_ack_payload(uint8_t pipe, const uint8_t* tx_payload, uint8_t length);

uint16_t nrf24_hal_read_rx_payload(uint8_t* rx_payload);

void nrf24_hal_power_down(void);
void nrf24_hal_power_up(void);
uint8_t nrf24_hal_is_data_available(void);
uint8_t nrf24_hal_is_data_available_from_pipe(uint8_t* pipe_no);
void nrf24_hal_set_payload_size(uint8_t payload_size);
uint8_t nrf24_hal_get_payload_size(void);
void nrf24_hal_set_irq_mode(nrf24_hal_irq_source_t int_source, uint8_t irq_state);
void nrf24_hal_set_crc_length(nrf24_hal_crc_length_t crc_length);
nrf24_hal_crc_length_t nrf24_hal_get_crc_length(void);
void nrf24_hal_disable_crc(void);
void nrf24_hal_open_pipe(nrf24_hal_pipe_t pipe_no, uint8_t auto_ack);
void nrf24_hal_close_pipe(nrf24_hal_pipe_t pipe_no);
void nrf24_hal_set_address_width(uint8_t address_width);
uint8_t nrf24_hal_get_address_width(void);
void nrf24_hal_set_auto_retransmit_delay(uint8_t ard);
void nrf24_hal_set_auto_retransmit_count(uint8_t arc);
void nrf24_hal_set_channel(uint8_t channel);
uint8_t nrf24_hal_get_channel(void);
void nrf24_hal_set_data_rate(nrf24_hal_datarate_t data_rate);
nrf24_hal_datarate_t nrf24_hal_get_data_rate(void);
void nrf24_hal_set_rf_output_power(nrf24_hal_rf_output_power_t rf_output_power);
void nrf24_hal_clear_irq_flag(nrf24_hal_irq_source_t int_source);
uint8_t nrf24_hal_get_clear_irq_flags(void);
uint8_t nrf24_hal_clear_irq_flags_get_status(void);
uint8_t nrf24_hal_get_irq_flags(void);
uint8_t nrf24_hal_get_rx_data_source(void);
uint8_t nrf24_hal_get_current_auto_retransmit_count(void);
void nrf24_hal_set_address_for_pipe(nrf24_hal_pipe_t pipe_no, const uint8_t* address);
void nrf24_hal_set_transmit_address(const uint8_t* address);
uint8_t nrf24_hal_get_rx_payload_width(uint8_t pipe_no);
void nrf24_hal_set_rx_payload_width(uint8_t pipe_no, uint8_t payload_width);
uint8_t nrf24_hal_is_tx_fifo_full(void);
uint8_t nrf24_hal_is_tx_fifo_empty(void);
uint8_t nrf24_hal_is_rx_fifo_full(void);
uint8_t nrf24_hal_is_rx_fifo_empty(void);
void nrf24_hal_set_dpl_for_pipe(nrf24_hal_pipe_t pipe, uint8_t is_enable);
void nrf24_hal_set_dpl_feature(uint8_t is_enable);
void nrf24_hal_set_ack_pkt_payload_feature(uint8_t is_enable);
uint8_t nrf24_hal_is_ack_pkt_payload_feature_enabled(void);
void nrf24_hal_set_no_ack_tx_payload_feature(uint8_t is_enable);
uint8_t                          nrf24_intl_read_register(uint8_t reg);
uint8_t                          nrf24_intl_read_multibyte_register(uint8_t reg, uint8_t* buffer, uint8_t len_to_read);
void nrf24_hal_print_details(uint8_t print_reg_value);
void nrf24_hal_reset();
uint8_t nrf24_intl_write_register(uint8_t reg, uint8_t value);
/*
The default state when the nrf24L01+ starts.

1. RX_DR (receiver Data Ready) interrupt enabled.
2. TX_DS (transmitter Data Send) interrupt enabled.
3. MAX_RT (Max ReTransmit) interrupt enabled.
4. 1 byte CRC enabled.
5. Starts in power down mode. In this mode the write register is allowed to configure the system.
6. Starts in transmitter mode.
7. Auto Ack enabled in all data pipes.
8. Data Pipe 0 & 1 are enabled.
9. 5 bytes address is set.
10. Auto Retransmit Delay is 250 uS.
11. Auto Retransmit Count is 3.
12. Rf channel is 2.
13. 2Mbps speed is selected.
14. 0dBm output power is selected.
15. Transmitter and Pipe 0 address is 0xE7E7E7E7E7
16. Pipe 1 address is 0xC2C2C2C2C2, Pipe 2 = 0xC2C2C2C2C3, Pipe 3 = 0xC2C2C2C2C4, Pipe 4 = 0xC2C2C2C2C5, Pipe 5 = 0xC2C2C2C2C6
17. Dynamic Payload Length is disabled in all pipes.
18. Dynamic Payload Length feature is disabled.
19. Ack Packet Payload feature is disabled.
20. W_TX_PAYLOAD_NOACK feature is disabled.

*/
#endif /* NRF24_H_ */
