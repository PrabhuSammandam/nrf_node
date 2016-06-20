/*
 * nrf24_link.h
 *
 * Created: 3/23/2016 7:46:41 PM
 *  Author: psammand
 */
#ifndef NRF24_LINK_H_
#define NRF24_LINK_H_

#include "nrf24_link_layer_config.h"
#include "nrf24_link_layer_params.h"
#include "nrf24_link_defs.h"

#define NRF24_LL_NO_TIMEOUT 0xFFFF
#define NRF24_LL_UNICAST    0
#define NRF24_LL_BROADCAST  1

/**
Dynamic protocol parameters.
*/
typedef enum
{
    NRF24_LL_PARAM_TX_ATTEMPTS,
    NRF24_LL_PARAM_RX_PIPES,
    NRF24_LL_PARAM_CRYPT_PIPES,
    NRF24_LL_PARAM_OUTPUT_POWER,
    NRF24_LL_PARAM_POWER_DOWN_IDLE_ENABLE,
    NRF24_LL_DYN_PARAM_SIZE
} gzll_dyn_params_t;

/**
Maximum values for dynamic protocol parameters.
*/
#define NRF24_LL_PARAMS_MAX         \
    {                               \
        0xffff, 0x3f, 0x3f, 0x03, 1 \
    }

void        nrf24_link_init(void);

void        nrf24_link_set_param(gzll_dyn_params_t param, uint16_t val);
uint16_t    nrf24_link_get_param_max(gzll_dyn_params_t param);
uint16_t    nrf24_link_get_param(gzll_dyn_params_t param);

void        nrf24_link_rx_start(void);
uint8_t     nrf24_link_rx_fifo_read(uint8_t* dst, uint8_t* length, uint8_t* pipe);

uint8_t     nrf24_link_ack_payload_write(const uint8_t* src, uint8_t length, uint8_t pipe);

void        nrf24_link_set_power(uint8_t power_on);
void        nrf24_link_set_nwk_id(uint16_t nwk_id);
void        nrf24_link_set_pan_id(uint16_t pan_id);

uint8_t     nrf24_link_tx_data(uint16_t pan_id, uint16_t nwk_id, const uint8_t* src, uint8_t length, uint8_t broad_cast, uint16_t timeout_ms);
uint8_t     nrf24_link_block_get_tx_status(void);

#endif /* NRF24_LINK_H_ */
