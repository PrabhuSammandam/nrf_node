/*
 * nrf24_link_layer_params.h
 *
 * Created: 3/24/2016 11:05:03 AM
 *  Author: psammand
 */
#ifndef NRF24_LINK_LAYER_PARAMS_H_
#define NRF24_LINK_LAYER_PARAMS_H_

/*nrf24 spi expects LSB byte first*/

#define NRF24_LL_DEFAULT_PAN_ID 0xFFFF
#define NRF24_LL_DEFAULT_NWK_ID 0xFFFF

#define NRF24_LL_DEFAULT_ADDRESS_PIPE0 \
    {                                  \
        0x00, 0x00, 0xFF, 0xFF, 0xE7   \
    }
#define NRF24_LL_DEFAULT_ADDRESS_PIPE1 \
    {                                  \
        0xFF, 0xFF, 0xE7, 0xE7, 0xE7   \
    }
#define NRF24_LL_DEFAULT_ADDRESS_PIPE2                  0xFE
#define NRF24_LL_DEFAULT_ADDRESS_PIPE3                  0xFD
#define NRF24_LL_DEFAULT_ADDRESS_PIPE4                  0xFC
#define NRF24_LL_DEFAULT_ADDRESS_PIPE5                  0xFB

#define NRF24_LL_DEFAULT_PARAM_TX_ATTEMPTS              5
#define NRF24_LL_DEFAULT_PARAM_RX_PIPES                 0x03
#define NRF24_LL_DEFAULT_PARAM_CRYPT_PIPES              0x00
#define NRF24_LL_DEFAULT_PARAM_OUTPUT_POWER             NRF24_HAL_RF_OUTPUT_POWER_0DBM
#define NRF24_LL_DEFAULT_PARAM_POWER_DOWN_IDLE_ENABLE   0
#define NRF24_LL_DEFAULT_AES_KEYS                                      \
    {                                                                  \
        {                                                              \
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16      \
        },                                                             \
        {                                                              \
            2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32 \
        }                                                              \
    }

#endif /* NRF24_LINK_LAYER_PARAMS_H_ */
