/*
 * nrf24_link_defs.h
 *
 * Created: 3/28/2016 4:22:41 PM
 *  Author: psammand
 */ 


#ifndef NRF24_LINK_DEFS_H_
#define NRF24_LINK_DEFS_H_

/*Number of bytes per packet exclusive the payload. That is: 1 preamble + 2 CRC + 5 address + ~1 byte packet control*/
#define NRF24_LL_CONST_BYTES_PER_PACKET (1 + 2 + 5 + 1)

#define NRF24_LL_CRC                    NRF24_HAL_CRC_16

#define NRF24_LL_ADDRESS_WIDTH          5

#define NRF24_LL_DEFAULT_CHANNEL	    76

/*Typical payload length. Used for calculating host mode 1 burst timing. */
#define NRF24_LL_TYPICAL_TX_PAYLOAD_LENGTH  15

/*
Typical transmit time including auto retry delay and 130 us radio startup.
Used for calculating host mode 1 burst behavior.
*/
#define NRF24_LL_TYPICAL_TX_PERIOD      (130 + ((NRF24_LL_CONST_BYTES_PER_PACKET + NRF24_LL_TYPICAL_TX_PAYLOAD_LENGTH) * NRF24_LL_US_PER_BYTE) + NRF24_LL_AUTO_RETR_DELAY)

#define NRF24_LL_MAX_FW_PAYLOAD_LENGTH  NRF24_LL_CONFIG_MAX_FW_PAYLOAD_LENGTH

#define NRF24_LL_MAX_ACK_PAYLOAD_LENGTH NRF24_LL_CONFIG_MAX_ACK_PAYLOAD_LENGTH

#define NRF24_LL_MAX_CRYPT_PIPES        NRF24_LL_CONFIG_MAX_CRYPT_PIPES

#if (NRF24_LL_MAX_FW_PAYLOAD_LENGTH > 32)
{
	#error "NRF24_LL_MAX_FW_PAYLOAD_LENGTH is limited to 32"
}
#endif

#if (NRF24_LL_MAX_ACK_PAYLOAD_LENGTH > 32)
{
	#error "NRF24_LL_MAX_ACK_PAYLOAD_LENGTH is limited to 32"
}
#endif

#if (NRF24_LL_MAX_FW_PAYLOAD_LENGTH > NRF24_LL_MAX_ACK_PAYLOAD_LENGTH)
#define NRF24_LL_MAX_PAYLOAD_LENGTH NRF24_LL_MAX_FW_PAYLOAD_LENGTH
#else
#define NRF24_LL_MAX_PAYLOAD_LENGTH NRF24_LL_MAX_ACK_PAYLOAD_LENGTH
#endif

#ifdef NRF24_LL_CONFIG_DATA_RATE_2MBPS

#define NRF24_LL_US_PER_BYTE            4
#define NRF24_LL_HAL_DATARATE           NRF24_HAL_DATA_RATE_2MBPS
#define NRF24_LL_HOST_CE_LOW_IDLE_DELAY 15  // Host active => Idle  delay. 9 * 50us = 500 us.
#if (NRF24_LL_MAX_ACK_PAYLOAD_LENGTH > 15)
#define NRF24_LL_AUTO_RETR_DELAY    500
#else
#define NRF24_LL_AUTO_RETR_DELAY    250
#endif

#endif /*NRF24_LL_CONFIG_DATA_RATE_2MBPS*/

#ifdef NRF24_LL_CONFIG_DATA_RATE_1MBPS
#define NRF24_LL_US_PER_BYTE            8
#define NRF24_LL_HAL_DATARATE           NRF24_HAL_DATA_RATE_1MBPS
#define NRF24_LL_HOST_CE_LOW_IDLE_DELAY 14  // Host cative -> Idle delay. 13 * 50us = 700 us.
#if (NRF24_LL_MAX_ACK_PAYLOAD_LENGTH > 5)
#define NRF24_LL_AUTO_RETR_DELAY    500
#else
#define NRF24_LL_AUTO_RETR_DELAY    250
#endif
#endif /*NRF24_LL_CONFIG_DATA_RATE_1MBPS*/

#ifdef NRF24_LL_CONFIG_DATA_RATE_250KBPS
#define NRF24_LL_US_PER_BYTE            32
#define NRF24_LL_HAL_DATARATE           NRF24_HAL_DATA_RATE_250KBPS
#define NRF24_LL_HOST_CE_LOW_IDLE_DELAY 37  // Host cative -> Idle delay. 36 * 50us = 1850 us.
#if (NRF24_LL_MAX_ACK_PAYLOAD_LENGTH == 0)
#define NRF24_LL_AUTO_RETR_DELAY    500
#elif (NRF24_LL_MAX_ACK_PAYLOAD_LENGTH < 8)
#define NRF24_LL_AUTO_RETR_DELAY    750
#elif (NRF24_LL_MAX_ACK_PAYLOAD_LENGTH < 16)
#define NRF24_LL_AUTO_RETR_DELAY    1000
#elif (NRF24_LL_MAX_ACK_PAYLOAD_LENGTH < 24)
#define NRF24_LL_AUTO_RETR_DELAY    1250
#else
#define NRF24_LL_AUTO_RETR_DELAY    1500
#endif
#endif /*NRF24_LL_CONFIG_DATA_RATE_250KBPS*/

#if (NRF24_LL_MAX_CRYPT_PIPES == 0)
#define NRF24_LL_MAX_CRYPT_PIPES_VAL    0x00
#elif (NRF24_LL_MAX_CRYPT_PIPES == 1)
#define NRF24_LL_MAX_CRYPT_PIPES_VAL    0x01
#elif (NRF24_LL_MAX_CRYPT_PIPES == 2)
#define NRF24_LL_MAX_CRYPT_PIPES_VAL    0x03
#elif (NRF24_LL_MAX_CRYPT_PIPES == 3)
#define NRF24_LL_MAX_CRYPT_PIPES_VAL    0x07
#elif (NRF24_LL_MAX_CRYPT_PIPES == 4)
#define NRF24_LL_MAX_CRYPT_PIPES_VAL    0x0f
#elif (NRF24_LL_MAX_CRYPT_PIPES == 5)
#define NRF24_LL_MAX_CRYPT_PIPES_VAL    0x1f
#elif (NRF24_LL_MAX_CRYPT_PIPES == 6)
#define NRF24_LL_MAX_CRYPT_PIPES_VAL    0x3f
#else
#error NRF24_LL_MAX_CRYPT_PIPES can not exceed 6.
#endif

#if (NRF24_LL_DEFAULT_PARAM_CRYPT_PIPES > NRF24_LL_MAX_CRYPT_PIPES_VAL)
#error NRF24_LL_MAX_CRYPT_PIPES not in accordance with NRF24_LL_DEFAULT_PARAM_CRYPT_PIPES.
#endif

#ifndef NRF24_LL_CONFIG_CRYTO_ENABLE
#if (NRF24_LL_MAX_CRYPT_PIPES > 0)
#warning NRF24_LL_MAX_CRYPT_PIPES > 0 but NRF24_LL_CONFIG_CRYTO_ENABLE is not defined.
#endif
#endif /*NRF24_LL_CRYPT_ENABLE*/

/*Final list of defines
1. NRF24_LL_MAX_FW_PAYLOAD_LENGTH               Max forward payload
2. NRF24_LL_MAX_ACK_PAYLOAD_LENGTH             Max ack payload
3. NRF24_LL_MAX_PAYLOAD_LENGTH                     Max payload
4. NRF24_LL_US_PER_BYTE
5. NRF24_LL_HAL_DATARATE
6. NRF24_LL_HOST_CE_LOW_IDLE_DELAY
7. NRF24_LL_AUTO_RETR_DELAY
8. NRF24_LL_MAX_CRYPT_PIPES
9. NRF24_LL_MAX_CRYPT_PIPES_VAL
*/

#endif /* NRF24_LINK_DEFS_H_ */