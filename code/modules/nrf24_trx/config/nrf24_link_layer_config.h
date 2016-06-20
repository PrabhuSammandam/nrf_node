/*
* nrf24_link_layer_config.h
*
* Created: 3/24/2016 10:09:55 AM
*  Author: psammand
*/
#ifndef NRF24_LINK_LAYER_CONFIG_H_
#define NRF24_LINK_LAYER_CONFIG_H_

/*
In general, a high data rate will lead to low power consumption as the
transmission time will be reduced. In addition the coexistence performance
is normally improved since shorter time on air reduces the risk of collisions.
A low data rate increases the receiver sensitivity, which will increase
the maximum range of the radio link.
*/
#define NRF24_LL_CONFIG_DATA_RATE_2MBPS

//#define NRF24_LL_CONFIG_DATA_RATE_1MBPS
//#define NRF24_LL_CONFIG_DATA_RATE_250KBPS
/*
NRF24_LL_CONFIG_CRYTO_ENABLE must be defined in order to use encryption at all.
*/
//#define NRF24_LL_CONFIG_CRYTO_ENABLE

/*Crypt bytes in payload config*/
//#define NRF24_LL_CONFIG_NUM_OF_CRYPTO_BYTES 5

/*
NRF24_LL_CONFIG_MAX_FW_PAYLOAD_LENGTH defines the maximum payload length for a
Device to Host packet (that is, a packet in "forward" direction).
The maximum value that can be set for GZLL_MAX_FW_PAYLOAD_LENGTH is 32 bytes
without encryption and 27 byte with encryption. 
(AES in counter mode as used by Gazell incurs a packet overhead of 5 bytes.)*/
#define NRF24_LL_CONFIG_MAX_FW_PAYLOAD_LENGTH   32

/*
NRF24_LL_CONFIG_MAX_ACK_PAYLOAD_LENGTH defines the maximum payload length for
a Host to Device acknowledgment packet. This parameter is also used to compute
the auto retransmit delay. A long ACK payload length will require a longer
retransmit delay between each transmission attempt from a Device.
*/
#define NRF24_LL_CONFIG_MAX_ACK_PAYLOAD_LENGTH  5

/*
NRF24_LL_CONFIG_MAX_CRYPT_PIPES defines the maximum number of pipes that
will be able to use AES encryption. The pipes that will be able to use
encryption is given as (0 <= pipe < GZLL_MAX_CRYPT_PIPES).

For example, if GZLL_MAX_CRYPT_PIPES is set to 3, pipe 0, 1 and 2 may use encryption.
*/
#define NRF24_LL_CONFIG_MAX_CRYPT_PIPES 0

#endif /* NRF24_LINK_LAYER_CONFIG_H_ */
