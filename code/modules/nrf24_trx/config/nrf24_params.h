#ifndef __NRF24_PARAMS_H__

#define ENABLE_MASK_RX_DR
#define ENABLE_MASK_TX_DS
#define ENABLE_MASK_MAX_RT

#define ENABLE_NO_CRC
#define ENABLE_2_BYTE_CRC
#define ENABLE_1_BYTE_CRC

#ifdef ENABLE_MASK_RX_DR
#define MASK_RX_DR_VAL  0x40
#else
#define MASK_RX_DR_VAL
#endif

#ifdef ENABLE_MASK_TX_DS
#define MASK_TX_DS_VAL  0x20
#else
#define MASK_TX_DS_VAL
#endif

#ifdef ENABLE_MASK_MAX_RT
#define MASK_MAX_RT_VAL 0x10
#else
#define MASK_MAX_RT_VAL
#endif

#if defined(ENABLE_NO_CRC)
#define CRC_VAL	0x00
#elif defined(ENABLE_1_BYTE_CRC)
#define CRC_VAL	0x08
#elif defined(ENABLE_2_BYTE_CRC)
#define CRC_VAL	0xC0
#endif

#define CONFIG_REG_VALUE (MASK_RX_DR_VAL|MASK_TX_DS_VAL|MASK_MAX_RT_VAL|CRC_VAL)

#endif /*__NRF24_PARAMS_H__*/
