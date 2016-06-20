#include <stdint.h>
#include "nrf24_hal.h"
#include "tm_phy.h"
#include "nrf24_link_layer.h"

/*
 * tm_phy.c
 *
 * Created: 3/30/2016 3:02:32 PM
 *  Author: psammand
 */

/*
ASCII code 179 = ?
ASCII code 180 = ? 
ASCII code 191 = ? 
ASCII code 192 = ? 
ASCII code 193 = ? 
ASCII code 194 = ? 
ASCII code 195 = ? 
ASCII code 196 = ? 
ASCII code 197 = ? 
ASCII code 217 = ? 
ASCII code 218 = ? 

*/

/*
????????????????????????????????
?                              ?
????????????????????????????????
?                              ?
????????????????????????????????
 */
typedef struct
{
    uint16_t    pan_id;
    uint16_t    nwk_addr;
    uint8_t     broadcast;
    uint8_t     size;
} phy_header_t;

#define PHY_STATE_INITIAL       0
#define PHY_STATE_IDLE          1
#define PHY_STATE_SLEEP         2
#define PHY_STATE_TX_WAIT_END   3

typedef struct
{
    uint8_t phy_state : 3;
    uint8_t phy_rx_state : 1;
    uint8_t reserved : 4;
    uint8_t phy_rx_buffer[32];
    uint8_t dst_addr[5];
} phy_inst_t;

static phy_inst_t   g_phy_inst = { .phy_rx_state = PHY_STATE_INITIAL, .dst_addr[4] = 0xE7 };

#define MSB(u16)    (((uint8_t*) &u16)[1])
#define LSB(u16)    (((uint8_t*) &u16)[0])

void PHY_Init(void)
{
    g_phy_inst.phy_rx_state = false;
    g_phy_inst.phy_state = PHY_STATE_IDLE;

    nrf24_link_init();
}

void PHY_SetRxState(bool    rx)
{
    if(g_phy_inst.phy_rx_state == rx) {
        return;
    }

    g_phy_inst.phy_rx_state = rx;

    if(rx) {
        nrf24_link_rx_start();
        g_phy_inst.phy_state = PHY_STATE_IDLE;
    }
    else {
        nrf24_link_set_power(false);
        g_phy_inst.phy_state = PHY_STATE_SLEEP;
    }
}

/**
 * \brief Sets the channel
 * 
 * \param channel channel to set valid range 0 - 125
 * 
 * \return void
 */
void PHY_SetChannel(uint8_t channel)
{
    nrf24_hal_set_channel(channel);
}

/*NWKADDR0, NWKADDR1, PAN0, PAN1, 0xE7*/
void PHY_SetPanId(uint16_t    panId)
{
    nrf24_link_set_pan_id(panId);
}

void PHY_SetShortAddr(uint16_t    addr)
{
    nrf24_link_set_nwk_id(addr);
}

void PHY_Sleep(void)
{
    PHY_SetRxState(false);
}

void PHY_Wakeup(void)
{
    PHY_SetRxState(true);
}

void PHY_DataReq(uint8_t*    data)
{
    phy_header_t*   phy_header;

    phy_header = (phy_header_t*)&data[0];
    g_phy_inst.dst_addr[0] = LSB(phy_header->nwk_addr);
    g_phy_inst.dst_addr[1] = MSB(phy_header->nwk_addr);
    g_phy_inst.dst_addr[2] = LSB(phy_header->pan_id);
    g_phy_inst.dst_addr[3] = MSB(phy_header->pan_id);

    data += sizeof(phy_header_t);

    nrf24_link_tx_data(phy_header->pan_id, phy_header->nwk_addr, &data[0], phy_header->size, phy_header->broadcast, NRF24_LL_NO_TIMEOUT);
}

void PHY_TaskHandler(void)
{
    if(PHY_STATE_SLEEP == g_phy_inst.phy_state) {
        return;
    }

    if(PHY_STATE_IDLE == g_phy_inst.phy_state) {
        uint8_t length;
        uint8_t pipe;

        if(!nrf24_link_rx_fifo_read(&g_phy_inst.phy_rx_buffer[0], &length, &pipe)) {
            PHY_DataInd_t   ind;

            ind.data = &g_phy_inst.phy_rx_buffer[0];
            ind.size = length;

            PHY_DataInd(&ind);
        }
    }
    else if(PHY_STATE_TX_WAIT_END == g_phy_inst.phy_state) {
        uint8_t status = nrf24_link_block_get_tx_status();

        if(status == 0) {
            status = PHY_STATUS_SUCCESS;
        }
        else if(status == 1) {
            status = PHY_STATUS_NO_ACK;
        }
        else {
            status = PHY_STATUS_ERROR;
        }

        PHY_SetRxState(true);

        PHY_DataConf(status);
    }
}
