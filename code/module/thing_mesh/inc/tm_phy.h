/*
 * tm_phy.h
 *
 * Created: 3/30/2016 2:59:49 PM
 *  Author: psammand
 */
#ifndef TM_PHY_H_
#define TM_PHY_H_

#include <stdint.h>
#include <stdbool.h>

/*- Types ------------------------------------------------------------------*/
typedef struct PHY_DataInd_t
{
    uint8_t*    data;
    uint8_t     size;
    uint8_t     lqi;
    int8_t      rssi;
} PHY_DataInd_t;

enum
{
    PHY_STATUS_SUCCESS                  = 0,
    PHY_STATUS_CHANNEL_ACCESS_FAILURE   = 1,
    PHY_STATUS_NO_ACK                   = 2,
    PHY_STATUS_ERROR                    = 3,
};

/*- Prototypes -------------------------------------------------------------*/
void PHY_Init(void);
void PHY_SetRxState(bool rx);
void PHY_SetChannel(uint8_t channel);
void PHY_SetPanId(uint16_t panId);
void PHY_SetShortAddr(uint16_t addr);
void PHY_Sleep(void);
void PHY_Wakeup(void);
void PHY_DataReq(uint8_t* data);
void PHY_DataConf(uint8_t status);
void PHY_DataInd(PHY_DataInd_t* ind);
void PHY_TaskHandler(void);

#endif /* TM_PHY_H_ */
