/*
 * nrf24_link.c
 *
 * Created: 3/23/2016 7:46:58 PM
 *  Author: psammand
 */
#include <string.h>
#include <avr/pgmspace.h>
#include "nrf24_hal.h"
#include "nrf24_link_layer.h"
#include "nrf24_port.h"

#ifndef _BV
#define _BV(bit)    (1 << (bit))
#endif

#ifndef MSB
#define MSB(u16)    (((uint8_t*) &u16)[1])
#endif

#ifndef LSB
#define LSB(u16)    (((uint8_t*) &u16)[0])
#endif

#define GZLL_SYSTEM_STATE_POWER_DOWN    0
#define GZLL_SYSTEM_STATE_STANDBY_I     1
#define GZLL_SYSTEM_STATE_STANDBY_II    2
#define GZLL_SYSTEM_STATE_RX            3
#define GZLL_SYSTEM_STATE_TX            4

static uint8_t  nrf24_link_intl_wait_for_tx_complete(uint16_t timeout_ms);
static void     nrf24_link_intl_set_radio_auto_retries(uint8_t current_retry_count);
static void     nrf24_link_intl_set_default_params(void);
static void     nrf24_link_intl_set_addr(uint8_t pipe_no, uint16_t pan_id, uint16_t nwk_id);

typedef struct
{
    uint8_t     state : 3;
    uint8_t     reserved : 5;
    uint16_t    gzll_dyn_params[NRF24_LL_DYN_PARAM_SIZE];
    uint16_t    pan_id;
    uint16_t    nwk_id;
} nrf24_link_instance_t;

nrf24_link_instance_t   g_inst;

/**
  * \brief 
  * The following default properties are enabled.
  * 1. set the CE to low to move in to standby mode.
  * 2. Enable dynamic payload length feature.
  * 3. Enable dynamic payload length for all pipes.
  * 4. Enable the Ack Payload feature.
  * 5. Set default pipe0 and pipe1 address.
  * 6. 
  * \return void
  */
void nrf24_link_init(void)
{
    uint8_t temp_address[NRF24_LL_ADDRESS_WIDTH] = NRF24_LL_DEFAULT_ADDRESS_PIPE1;

    nrf24_link_intl_set_default_params();

    g_inst.state = GZLL_SYSTEM_STATE_POWER_DOWN;
    g_inst.pan_id = NRF24_LL_DEFAULT_PAN_ID;
    g_inst.nwk_id = NRF24_LL_DEFAULT_NWK_ID;

    nrf24_hal_init();

    CE_LOW();
    nrf24_hal_power_up();

    /*enable dynamic payload and W_TX_PAYLOAD_NOACK command*/
    nrf24_write_register(FEATURE_REG, (_BV(EN_DPL) | _BV(EN_DYN_ACK)));

    /*enable dynamic payload in all pipes*/
    nrf24_write_register(DYNAMIC_PAYLOAD_ENABLE_REG, 0x3F);

    /*set address width defined in config file*/
    nrf24_write_register(SETUP_ADDRESS_WIDTH_REG, NRF24_LL_ADDRESS_WIDTH - 2);

    /*set default channel*/
    nrf24_write_register(RF_CHANNEL_REG, NRF24_LL_DEFAULT_CHANNEL);

    /* set data rate and the output power*/
    nrf24_write_register(RF_SETUP_REG, (NRF24_LL_HAL_DATARATE | ((uint8_t) g_inst.gzll_dyn_params[NRF24_LL_PARAM_OUTPUT_POWER])));

    /* set CRC length */
    nrf24_hal_set_crc_length(NRF24_LL_CRC);

    nrf24_link_set_pan_id(g_inst.pan_id);
    nrf24_link_set_nwk_id(g_inst.nwk_id);

    nrf24_hal_set_address_for_pipe(NRF24_HAL_PIPE_1, &temp_address[0]);

    nrf24_hal_get_clear_irq_flags();

    nrf24_hal_flush_rx_fifo();
    nrf24_hal_flush_tx_fifo();

    if(g_inst.gzll_dyn_params[NRF24_LL_PARAM_ENABLE_POWER_DOWN_IDLE] != 1) {
        g_inst.state = GZLL_SYSTEM_STATE_STANDBY_I;
    }
}

void nrf24_link_set_param(gzll_dyn_params_t   param,
                          uint16_t            val)
{
    if(param < NRF24_LL_DYN_PARAM_SIZE) {
        g_inst.gzll_dyn_params[param] = val;

        switch(param)
        {
            case NRF24_LL_PARAM_ENABLE_POWER_DOWN_IDLE:
                if(val == 1) {
                    nrf24_hal_power_up();
                    g_inst.state = GZLL_SYSTEM_STATE_POWER_DOWN;
                }
                break;

            case NRF24_LL_PARAM_OUTPUT_POWER:
                nrf24_write_register(RF_SETUP_REG, (NRF24_LL_HAL_DATARATE | ((uint8_t) g_inst.gzll_dyn_params[NRF24_LL_PARAM_OUTPUT_POWER])));
                break;

            default:
                break;
        }
    }
}

uint16_t nrf24_link_get_param_max(gzll_dyn_params_t   param)
{
    uint16_t    param_max[NRF24_LL_DYN_PARAM_SIZE] = NRF24_LL_PARAMS_MAX;

    return param_max[param];
}

uint16_t nrf24_link_get_param(gzll_dyn_params_t   param)
{
    if(param < NRF24_LL_DYN_PARAM_SIZE) {
        return g_inst.gzll_dyn_params[param];
    }

    return 0;
}

void nrf24_link_set_pan_id(uint16_t    pan_id)
{
    g_inst.pan_id = pan_id;
    nrf24_link_intl_set_addr(NRF24_HAL_PIPE_0, pan_id, g_inst.nwk_id);
}

void nrf24_link_set_nwk_id(uint16_t    nwk_id)
{
    g_inst.nwk_id = nwk_id;
    nrf24_link_intl_set_addr(NRF24_HAL_PIPE_0, g_inst.pan_id, nwk_id);
}

static void nrf24_link_intl_set_addr(uint8_t     pipe_no,
                                     uint16_t    pan_id,
                                     uint16_t    nwk_id)
{
    uint8_t node_address[NRF24_LL_ADDRESS_WIDTH];

    node_address[0] = LSB(nwk_id);
    node_address[1] = MSB(nwk_id);
    node_address[2] = LSB(pan_id);
    node_address[3] = MSB(pan_id);
    node_address[4] = 0xE7;

    nrf24_hal_set_address_for_pipe(pipe_no, &node_address[0]);
}

void nrf24_link_set_power(uint8_t power_on)
{
    if(power_on) {
        nrf24_hal_power_up();
    }
    else {
        nrf24_hal_power_down();
    }
}

uint8_t nrf24_link_rx_fifo_read(uint8_t*    dst,
                                uint8_t*    length,
                                uint8_t*    pipe)
{
    uint16_t    pipe_and_length;

    if(!nrf24_hal_is_rx_fifo_empty()) {
        *length = nrf24_hal_read_rx_payload_width();

        if(*length <= 32) {

            pipe_and_length = nrf24_hal_read_rx_payload(dst);

            if(pipe != 0) {
                *pipe = ((pipe_and_length >> 8) & 0xFF);
            }

            return 0;
        }
        else {
            *length = 0;
            nrf24_hal_flush_rx_fifo();
        }

        nrf24_hal_get_clear_irq_flags();
    }

    return 1;
}

uint8_t nrf24_link_ack_payload_write(const uint8_t*  src,
                                     uint8_t         length,
                                     uint8_t         pipe)
{
    if(length == 0 || (length > NRF24_LL_MAX_ACK_PAYLOAD_LENGTH) || nrf24_hal_is_tx_fifo_full()) {
        return 0;                       // ACK payload not written
    }

    nrf24_hal_write_ack_payload(pipe, src, length);
    return 1;                           // ACK payload successfully written
}

void nrf24_link_rx_start(void)
{
    uint8_t i;

    if(g_inst.state == GZLL_SYSTEM_STATE_RX) {
        return;
    }
    else if(g_inst.state == GZLL_SYSTEM_STATE_POWER_DOWN) {
        nrf24_hal_power_up();           // after this it moves to standby-I mode
    }

    CE_LOW();        // after this it moves to standby-I mode

    /* Restore pipe 0 address (this may have been altered during transmission) */
    nrf24_link_intl_set_addr(NRF24_HAL_PIPE_0, g_inst.pan_id, g_inst.nwk_id);

    /* close all pipes*/
    nrf24_hal_close_pipe(NRF24_HAL_PIPE_ALL);

    /* Enable the receive pipes selected by gzll_set_param() */
    for(i = 0; i < 6; i++) {
        if(g_inst.gzll_dyn_params[NRF24_LL_PARAM_RX_PIPES] & (1 << i)) {
            nrf24_hal_open_pipe((nrf24_hal_pipe_t) i, 1);
        }
    }

    nrf24_hal_set_operation_mode(NRF24_HAL_OP_MODE_RX);
    CE_HIGH();
    nrf24_port_delay_us(130);

    g_inst.state = GZLL_SYSTEM_STATE_RX;
}

uint8_t nrf24_link_tx_data(uint16_t        pan_id,
                           uint16_t        nwk_id,
                           const uint8_t*  src,
                           uint8_t         length,
                           uint8_t         broad_cast,
                           uint16_t        timeout_ms)
{
    uint8_t success = 1;
    uint8_t current_retry_count = 0;

    /* Length check to prevent memory corruption. (Note, assertion will capture this as well). */
    if(length == 0 || length > NRF24_LL_MAX_FW_PAYLOAD_LENGTH) {
        return 0;
    }

    if(g_inst.state == GZLL_SYSTEM_STATE_POWER_DOWN) {
        CE_LOW();             
        nrf24_hal_power_up(); // after this it moves to standby-I mode
    }
    else if(g_inst.state == GZLL_SYSTEM_STATE_RX) {
        CE_LOW();    // after this it moves to standby-I mode
    }

    nrf24_link_intl_set_addr(NRF24_HAL_PIPE_0, pan_id, nwk_id);
    nrf24_link_intl_set_addr(NRF24_HAL_PIPE_TX, pan_id, nwk_id);

    nrf24_hal_open_pipe(NRF24_HAL_PIPE_0, 1);

    current_retry_count = g_inst.gzll_dyn_params[NRF24_LL_PARAM_TX_ATTEMPTS];
    nrf24_link_intl_set_radio_auto_retries(current_retry_count);

    nrf24_hal_set_operation_mode(NRF24_HAL_OP_MODE_TX);

    if(broad_cast == NRF24_LL_BROADCAST) {
        nrf24_hal_write_tx_payload_noack(src, length);
    }
    else {
        nrf24_hal_write_tx_payload(src, length);
    }

    CE_HIGH();

    g_inst.state = GZLL_SYSTEM_STATE_TX;

    success = nrf24_link_intl_wait_for_tx_complete(timeout_ms);

    CE_LOW();
    nrf24_link_rx_start();

    return success;
}

uint8_t nrf24_link_block_get_tx_status(void)
{
    return nrf24_link_intl_wait_for_tx_complete(NRF24_LL_NO_TIMEOUT);
}

static uint8_t nrf24_link_intl_wait_for_tx_complete(uint16_t    timeout_ms)
{
    uint8_t     status;
    uint8_t     success = 0;
    uint32_t    start_time_ms;
    uint16_t    current_retry_count = g_inst.gzll_dyn_params[NRF24_LL_PARAM_TX_ATTEMPTS];

	start_time_ms = nrf24_port_current_time_ms();

    while(1) {
        if((timeout_ms != 0xFFFF) && ((nrf24_port_current_time_ms() - start_time_ms) > timeout_ms)) {
            success = 1;
            break;
        }

        status = nrf24_hal_get_irq_flags();

        if((status & _BV(TX_DS)) != 0) {
            success = 0;
            break;
        }

        if((status & _BV(MAX_RT)) != 0) {
            printf_P(PSTR("max rt\n"));

            current_retry_count -= (nrf24_hal_get_current_auto_retransmit_count() + 1);

            if(current_retry_count <= 0) {
                success = 2;
                break;
            }
            else {
                printf_P(PSTR("resend arc %d\n"), current_retry_count);
                nrf24_hal_clear_irq_flag(NRF24_HAL_IRQ_MAX_RT);
                nrf24_link_intl_set_radio_auto_retries(current_retry_count);        // Continue retransmits on same channel
                nrf24_hal_chip_enable_pulse();
            }
        }
    }

    nrf24_hal_clear_irq_flag(NRF24_HAL_IRQ_TX_DS);
    nrf24_hal_clear_irq_flag(NRF24_HAL_IRQ_MAX_RT);

    printf_P(PSTR("success %d\n"), success);
    return success;
}

static void nrf24_link_intl_set_radio_auto_retries(uint8_t current_retry_count)
{
    nrf24_hal_set_auto_retransmit_count((current_retry_count > 15) ? 15 : current_retry_count - 1);
    nrf24_hal_set_auto_retransmit_delay(((uint16_t)NRF24_LL_AUTO_RETR_DELAY) >> 8);
}

void nrf24_port_radio_isr_function()
{
	uint8_t status;

	status = nrf24_hal_get_clear_irq_flags();

	/*received some packet*/
	if((status & _BV(RX_DR)))
	{
	}

	/*transmitted the packet*/
	if((status & _BV(TX_DS)))
	{
	}

	/*transmission failed*/
	if((status & _BV(MAX_RT)))
	{
	}
}

static void nrf24_link_intl_set_default_params(void)
{
    g_inst.gzll_dyn_params[NRF24_LL_PARAM_TX_ATTEMPTS] = NRF24_LL_DEFAULT_PARAM_TX_ATTEMPTS;
    g_inst.gzll_dyn_params[NRF24_LL_PARAM_RX_PIPES] = NRF24_LL_DEFAULT_PARAM_RX_PIPES;
    g_inst.gzll_dyn_params[NRF24_LL_PARAM_CRYPT_PIPES] = NRF24_LL_DEFAULT_PARAM_CRYPT_PIPES;
    g_inst.gzll_dyn_params[NRF24_LL_PARAM_OUTPUT_POWER] = NRF24_LL_DEFAULT_PARAM_OUTPUT_POWER;
    g_inst.gzll_dyn_params[NRF24_LL_PARAM_ENABLE_POWER_DOWN_IDLE] = NRF24_LL_DEFAULT_PARAM_POWER_DOWN_IDLE_ENABLE;
}
