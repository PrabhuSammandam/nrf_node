/*
* nrf24_hal.c
*
* Created: 20-03-2016 21:02:55
*  Author: prabhu sammandam
*/
#include "utils/compiler.h"
#include "nrf24_hal.h"
#include "nrf24l01.h"
#include "nrf24_port.h"

//@}
/**
 * Typedef for the SETUP_RETR register. Contains all the bit addressable
 * settings in the bits struct and the value sent to the radio in the uint8_t
 */
typedef union
{
    uint8_t value;
    struct
    {
        uint8_t arc : 4;
        uint8_t ard : 4;
    } bits;
} setup_retr_t;

/**
 * Typedef for the CONFIG register. Contains all the bit addressable
 * settings in the bits struct and the value sent to the radio in the uint8_t
 */
typedef union
{
    uint8_t value;
    struct
    {
        uint8_t prim_rx : 1;
        uint8_t pwr_up : 1;
        uint8_t crc0 : 1;
        uint8_t en_crc : 1;
        uint8_t mask_max_rt : 1;
        uint8_t mask_tx_ds : 1;
        uint8_t mask_rx_dr : 1;
        const   uint8_t : 1;
    } bits;
} config_t;

/**
 * Typedef for the RF_SETUP register. Contains all the bit addressable
 * settings in the bits struct and the value sent to the radio in the uint8_t
 */
typedef union
{
    uint8_t value;
    struct
    {
        const   uint8_t : 1;
        uint8_t rf_pwr : 2;
        uint8_t rf_dr_high : 1;
        uint8_t pll_lock : 1;
        uint8_t rf_dr_low : 1;
        const   uint8_t : 1;
        uint8_t cont_wave : 1;
    } bits;
} rf_setup_t;

#ifndef _BV
#define _BV(bit)    (1 << (bit))
#endif

#define MAX_VALUE(a, b) (a > b ? a : b)
#define MIN_VALUE(a, b) (a < b ? a : b)

static const char data_rate_str_0[] PROGMEM = "1Mbps";
static const char data_rate_str_1[] PROGMEM = "2Mbps";
static const char data_rate_str_2[] PROGMEM = "250Kbps";
static const char* const data_rate_str_P[] PROGMEM = {data_rate_str_0, data_rate_str_1, data_rate_str_2};

static const char crc_length_str_0[] PROGMEM = "Disabled";
static const char crc_length_str_1[] PROGMEM = "8 Bits";
static const char crc_length_str_2[] PROGMEM = "16  Bits";
static const char* const crc_length_str_P[] PROGMEM = {crc_length_str_0, crc_length_str_1, crc_length_str_2};

static const char out_power_str_0[] PROGMEM = "MIN";
static const char out_power_str_1[] PROGMEM = "LOW";
static const char out_power_str_2[] PROGMEM = "HIGH";
static const char out_power_str_3[] PROGMEM = "MAX";
static const char* const out_power_str_P[] PROGMEM = {out_power_str_0, out_power_str_1, out_power_str_2, out_power_str_3};

static const char power_up_P[] PROGMEM = "UP";
static const char power_down_P[] PROGMEM = "DOWN";
static const char* const power_str_table_P[] PROGMEM = {power_down_P, power_up_P};

static const char device_operating_mode_rx_P[] PROGMEM = "RX";
static const char device_operating_mode_tx_P[] PROGMEM = "TX";
static const char* const device_operating_mode_str_table_P[] PROGMEM = {device_operating_mode_tx_P, device_operating_mode_rx_P};

static const char enabled_str_P[] PROGMEM = "Enabled";
static const char disabled_str_P[] PROGMEM = "Disabled";
static const char* const en_dis_str_table_P[] PROGMEM = {disabled_str_P, enabled_str_P};


/******************************************************************************************/
__always_inline static void             chip_select_low(void);
__always_inline static void             chip_select_high(void);
//static uint8_t                          nrf24_intl_read_register(uint8_t reg);
//static uint8_t                          nrf24_intl_read_multibyte_register(uint8_t reg, uint8_t* buffer, uint8_t len_to_read);
uint8_t                          nrf24_intl_write_register(uint8_t reg, uint8_t value);
static uint8_t                          nrf24_intl_write_multibyte_register(uint8_t reg, const uint8_t* buffer, uint8_t len_to_write);
static uint8_t                          nrf24_intl_send_command(uint8_t command);
static uint8_t                          nrf24_intl_get_status_value(void);
__always_inline static inline void      clear_bit(uint8_t reg, uint8_t bit);
__always_inline static inline void      set_bit(uint8_t reg, uint8_t bit);
__always_inline static inline uint8_t   is_bit_set(uint8_t reg, uint8_t bit);

/****************************PRIVATE FUNCTIONS*********************************************/

/*chip select pin is used for the spi communication*/
__always_inline static void chip_select_low(void)
{
    nrf24_port_set_chip_select(NRF24_HAL_CHIP_SELECT_VALUE_LOW);
    //nrf24_port_delay_us(5);
}

/*chip select pin is used for the spi communication*/
__always_inline static void chip_select_high(void)
{
    nrf24_port_set_chip_select(NRF24_HAL_CHIP_SELECT_VALUE_HIGH);
    //nrf24_port_delay_us(5);
}

uint8_t nrf24_intl_read_register(uint8_t reg)
{
    uint8_t result;

    chip_select_low();
    nrf24_port_spi_write_byte(reg);
    result = nrf24_port_spi_write_byte(NOP);
    chip_select_high();

    return result;
}

uint8_t nrf24_intl_read_multibyte_register(uint8_t     reg,
                                                  uint8_t*    buffer,
                                                  uint8_t     len_to_read)
{
    uint8_t status;

    chip_select_low();
    status = nrf24_port_spi_write_byte(reg);
    while(len_to_read--) {
        *buffer++ = nrf24_port_spi_write_byte(NOP);
    }

    chip_select_high();

    return status;
}

uint8_t nrf24_intl_write_register(uint8_t reg,
                                         uint8_t value)
{
    uint8_t status;

    chip_select_low();
    status = nrf24_port_spi_write_byte(W_REGISTER + reg);
    nrf24_port_spi_write_byte(value);
    chip_select_high();

    return status;
}

/*
The transmit payload is of three types
1. Normal Payload : W_TX_PAYLOAD
2. No Ack Payload : W_TX_PAYLOAD_NOACK
3. Ack Pkt Payload : W_ACK_PAYLOAD | pipe
*/
static uint8_t nrf24_intl_write_multibyte_register(uint8_t         reg,
                                                   const uint8_t*  buffer,
                                                   uint8_t         len_to_write)
{
    uint8_t status;

    chip_select_low();
    status = nrf24_port_spi_write_byte(reg);
    while(len_to_write--) {
        nrf24_port_spi_write_byte(*buffer++);
    }

    chip_select_high();

    return status;
}

static uint8_t nrf24_intl_send_command(uint8_t command)
{
    uint8_t status;

    chip_select_low();
    status = nrf24_port_spi_write_byte(command);
    chip_select_high();

    return status;
}

static uint8_t nrf24_intl_get_status_value(void)
{
    return nrf24_intl_send_command(NOP);
}

__always_inline static inline void clear_bit(uint8_t reg,
                                             uint8_t bit)
{
    nrf24_intl_write_register(reg, nrf24_intl_read_register(reg) &~_BV(bit));
}

__always_inline static inline void set_bit(uint8_t reg,
                                           uint8_t bit)
{
    nrf24_intl_write_register(reg, nrf24_intl_read_register(reg) | _BV(bit));
}

__always_inline static inline uint8_t is_bit_set(uint8_t reg,
                                                 uint8_t bit)
{
    uint8_t value = nrf24_intl_read_register(reg);

    return ((value & _BV(bit)) != 0);
}

#define SET_BIT(REG, BIT)   nrf24_intl_write_register(REG, nrf24_intl_read_register(REG) | _BV(BIT));

/***************************************PUBLIC FUNCTIONS*********************************************/

/*chip enable pins activates RX or TX mode*/
void nrf24_hal_chip_enable_low(void)
{
    nrf24_port_set_chip_enable(NRF24_HAL_CHIP_ENABLE_VALUE_LOW);
}

/*chip enable pins activates RX or TX mode*/
void nrf24_hal_chip_enable_high(void)
{
    nrf24_port_set_chip_enable(NRF24_HAL_CHIP_ENABLE_VALUE_HIGH);
}

/*
Chip Enable pulse is mainly used for triggering the transmit payload in PTX mode.
*/
void nrf24_hal_chip_enable_pulse(void)
{
    nrf24_hal_chip_enable_high();
    nrf24_port_delay_us(10);
    nrf24_hal_chip_enable_low();
}

uint8_t nrf24_hal_flush_rx_fifo(void)
{
    return nrf24_intl_send_command(FLUSH_RX);
}

uint8_t nrf24_hal_flush_tx_fifo(void)
{
    return nrf24_intl_send_command(FLUSH_TX);
}

void nrf24_hal_reuse_tx_payload(void)
{
    nrf24_intl_write_register(STATUS_REG, _BV(MAX_RT)); //Clear max retry flag
    nrf24_intl_send_command(REUSE_TX_PL);
}

uint8_t nrf24_hal_read_rx_payload_width(void)
{
    return nrf24_intl_read_register(R_RX_PL_WID);
}

/*
* CONFIG register related functions
*/
void nrf24_hal_init(void)
{
    nrf24_port_init();
}

uint8_t nrf24_hal_start(void)
{
    uint8_t setup = 0;

    chip_select_high();
    nrf24_hal_chip_enable_low();

    // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    nrf24_port_delay_ms(5);

    nrf24_hal_set_crc_length(NRF24_HAL_CRC_16);

    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See documentation for a more complete explanation.
    nrf24_hal_set_auto_retransmit_count(15);
    nrf24_hal_set_auto_retransmit_delay(NRF24_HAL_ARD_1500_US);

    nrf24_hal_close_pipe(NRF24_HAL_PIPE_ALL);

    nrf24_hal_set_address_width(5);

    // Then set the data rate to the slowest (and most reliable) speed supported by all hardware.
    nrf24_hal_set_data_rate(NRF24_HAL_DATA_RATE_1MBPS);

    // Reset current status
    // Notice reset and flush is the last thing we do
    nrf24_hal_get_clear_irq_flags();

    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent spectrum.
    nrf24_hal_set_channel(76);

    // Flush buffers
    nrf24_hal_flush_rx_fifo();
    nrf24_hal_flush_tx_fifo();

    nrf24_hal_power_up();           //Power up by default when begin() is called
    nrf24_hal_set_tx_mode();

    setup = nrf24_intl_get_status_value();

    // if setup is 0 or ff then there was no response from module
    return (setup != 0 && setup != 0xFF);
}

void nrf24_hal_set_tx_mode(void)
{
    //uint32_t    transmit_receive_delay_us = g_nrf24_instance.transmit_receive_delay_us;
    /* if the system is in power down mode as in case system starts just now. No harm in calling this api
	 * because it checks the power up bit and if not set then only sets it.
	 */
    nrf24_hal_power_up();

    /* if the system is in receiver mode previously then first move to standby-I mode*/
    nrf24_hal_chip_enable_low();

    //nrf24_port_delay_us(transmit_receive_delay_us);
    // if ack payload enabled then clear the transmitter payload
    if(nrf24_hal_is_ack_pkt_payload_feature_enabled()) {

        //nrf24_port_delay_us(transmit_receive_delay_us);                     //200
        nrf24_hal_flush_tx_fifo();
    }

    clear_bit(CONFIG_REG, PRIMARY_RX_BIT);
}

void nrf24_hal_set_rx_mode(void)
{
    /* if the system is in power down mode as in case system starts just now. No harm in calling this api
	 * because it checks the power up bit and if not set then only sets it.
	 */
    nrf24_hal_power_up();

    /*after above api call the chip is in standby-I state*/
    SET_BIT(CONFIG_REG, PRIMARY_RX_BIT);

    /*clear all the interrupts*/
    nrf24_hal_get_clear_irq_flags();

    /*Move to receiver mode*/
    nrf24_hal_chip_enable_high();

    /*delay for 130us for rx settling time*/
    nrf24_port_delay_us(130);

    if(nrf24_hal_is_ack_pkt_payload_feature_enabled()) {
        nrf24_hal_flush_tx_fifo();
    }
}

void nrf24_hal_set_operation_mode(nrf24_nrf_operation_mode_t  op_mode)
{
    if(op_mode == NRF24_HAL_OP_MODE_RX) {
        SET_BIT(CONFIG_REG, PRIMARY_RX_BIT);
    }
    else {
        clear_bit(CONFIG_REG, PRIMARY_RX_BIT);
    }
}

void nrf24_hal_write_tx_payload(const uint8_t*  tx_payload,
                                uint8_t         length)
{
    nrf24_intl_write_multibyte_register(W_TX_PAYLOAD, tx_payload, length);
}

void nrf24_hal_write_tx_payload_noack(const uint8_t*  tx_payload,
                                      uint8_t         length)
{
    nrf24_intl_write_multibyte_register(W_TX_PAYLOAD_NOACK, tx_payload, length);
}

void nrf24_hal_write_ack_payload(uint8_t         pipe,
                                 const uint8_t*  tx_payload,
                                 uint8_t         length)
{
    nrf24_intl_write_multibyte_register(W_ACK_PAYLOAD | pipe, tx_payload, length);
}

uint16_t nrf24_hal_read_rx_payload(uint8_t*    rx_payload)
{
    uint8_t buffer_length;
    uint8_t pipe;

    /* first get the pipe where the data is received*/
    pipe = nrf24_hal_get_rx_data_source();

    if(pipe < 7U) {

        /*get the received payload width by reading the R_RX_PL_WID register*/
        buffer_length = nrf24_hal_read_rx_payload_width();

        /*finally read the payload from rx fifo*/
        nrf24_intl_read_multibyte_register(R_RX_PAYLOAD, rx_payload, buffer_length);
    }
    else {

        /*if pipe is >= 7 then the rx fifo is empty*/
        pipe = 0xFF;
        buffer_length = 0U;
    }

    return (((uint16_t) pipe << 8) | buffer_length);
}

void nrf24_hal_power_down(void)
{
    /*first set CE to low, since in any mode RX or TX setting CE to low will move to standby-I mode, then from
	* standyby-I mode move to power down mode by setting the PWR_UP bit*/
    nrf24_hal_chip_enable_low();    // Guarantee CE is low on powerDown
    clear_bit(CONFIG_REG, POWER_UP_BIT);
}

void nrf24_hal_power_up(void)
{
    // if not powered up then power up and wait for the radio to initialize
    //if(!is_bit_set(CONFIG_REG, POWER_UP_BIT)) {
        set_bit(CONFIG_REG, POWER_UP_BIT);

        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
        // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
        // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 data sheet
        nrf24_port_delay_ms(5);
    //}
}

uint8_t nrf24_hal_is_data_available(void)
{
    return nrf24_hal_is_data_available_from_pipe(0);
}

uint8_t nrf24_hal_is_data_available_from_pipe(uint8_t*    pipe_no)
{
    if(!nrf24_hal_is_rx_fifo_empty()) {
        if(pipe_no) {
            *pipe_no = nrf24_hal_get_rx_data_source();
        }

        return 1;
    }

    return 0;
}

void nrf24_hal_set_irq_mode(nrf24_hal_irq_source_t  int_source,
                            uint8_t                 irq_state)
{
    config_t    config;
    config.value = nrf24_intl_read_register(CONFIG_REG);

    if(int_source == NRF24_HAL_IRQ_MAX_RT) {
        config.bits.mask_max_rt = irq_state ? 0U : 1U;
    }
    else if(int_source == NRF24_HAL_IRQ_TX_DS) {
        config.bits.mask_tx_ds = irq_state ? 0U : 1U;
    }
    else if(int_source == NRF24_HAL_IRQ_RX_DR) {
        config.bits.mask_rx_dr = irq_state ? 0U : 1U;
    }

    nrf24_intl_write_register(CONFIG_REG, config.value);
}

void nrf24_hal_set_crc_length(nrf24_hal_crc_length_t  crc_length)
{
    config_t    config;
    config.value = nrf24_intl_read_register(CONFIG_REG);

    // switch uses RAM (evil!)
    if(crc_length == NRF24_HAL_CRC_DISABLED) {
        config.bits.en_crc = 0U;
    }
    else if(crc_length == NRF24_HAL_CRC_8) {
        config.bits.en_crc = 1U;
        config.bits.crc0 = 0U;
    }
    else {
        config.bits.en_crc = 1U;
        config.bits.crc0 = 1U;
    }

    nrf24_intl_write_register(CONFIG_REG, config.value);

    NRF24_DEBUG_PRINTF(printf_P(PSTR("S=> CRC 0x%x\n"), crc_length));
}

nrf24_hal_crc_length_t nrf24_hal_get_crc_length(void)
{
    config_t    config;
    config.value = nrf24_intl_read_register(CONFIG_REG);

    if(!config.bits.en_crc) {
        return NRF24_HAL_CRC_DISABLED;
    }
    else if(config.bits.crc0) {
        return NRF24_HAL_CRC_16;
    }

    return NRF24_HAL_CRC_8;
}

void nrf24_hal_disable_crc(void)
{
    clear_bit(CONFIG_REG, ENABLE_CRC_BIT);
}

void nrf24_hal_open_pipe(nrf24_hal_pipe_t    pipe_no,
                         uint8_t             auto_ack)
{
    if(pipe_no == NRF24_HAL_PIPE_ALL) {
        nrf24_intl_write_register(ENABLE_RX_ADDR_REG, 0x3F);
        nrf24_intl_write_register(ENABLE_AUTO_ACK_REG, (auto_ack) ? 0x3F : 0x00);
    }
    else {
        SET_BIT(ENABLE_RX_ADDR_REG, pipe_no);

        if(auto_ack) {
            SET_BIT(ENABLE_AUTO_ACK_REG, pipe_no);
        }
        else {
            clear_bit(ENABLE_AUTO_ACK_REG, pipe_no);
        }
    }
}

void nrf24_hal_close_pipe(nrf24_hal_pipe_t    pipe_no)
{
    if(pipe_no == NRF24_HAL_PIPE_ALL) {
        nrf24_intl_write_register(ENABLE_RX_ADDR_REG, 0);
        nrf24_intl_write_register(ENABLE_AUTO_ACK_REG, 0);
    }
    else {
        clear_bit(ENABLE_RX_ADDR_REG, pipe_no);
        clear_bit(ENABLE_AUTO_ACK_REG, pipe_no);
    }
}

void nrf24_hal_set_address_width(uint8_t address_width)
{
    // -2 because to convert it in to binary. Eg aw = 5 => 5-2 => 3 => 0b11
    nrf24_intl_write_register(SETUP_ADDRESS_WIDTH_REG, address_width - 2);
}

uint8_t nrf24_hal_get_address_width(void)
{
    return nrf24_intl_read_register(SETUP_ADDRESS_WIDTH_REG) + 2U;
}

void nrf24_hal_set_auto_retransmit_delay(uint8_t ard)
{
    setup_retr_t    retry_value;

    retry_value.value = nrf24_intl_read_register(SETUP_RETRANSMISSION_REG);
    retry_value.bits.ard = ard;
    nrf24_intl_write_register(SETUP_RETRANSMISSION_REG, retry_value.value);
}

void nrf24_hal_set_auto_retransmit_count(uint8_t arc)
{
    setup_retr_t    retry_value;

    retry_value.value = nrf24_intl_read_register(SETUP_RETRANSMISSION_REG);
    retry_value.bits.arc = arc;
    nrf24_intl_write_register(SETUP_RETRANSMISSION_REG, retry_value.value);
}

void nrf24_hal_set_channel(uint8_t channel)
{
    nrf24_intl_write_register(RF_CHANNEL_REG, MIN_VALUE(channel, 125));
}

uint8_t nrf24_hal_get_channel(void)
{
    return nrf24_intl_read_register(RF_CHANNEL_REG);
}

void nrf24_hal_set_data_rate(nrf24_hal_datarate_t    data_rate)
{
    rf_setup_t  rf_setup;
    rf_setup.value = nrf24_intl_read_register(RF_SETUP_REG);

    /*default 1MPS*/
    rf_setup.bits.rf_dr_low = 0U;
    rf_setup.bits.rf_dr_high = 0U;

    //g_nrf24_instance.transmit_receive_delay_us = 85;
    if(data_rate == NRF24_HAL_DATA_RATE_250KBPS) {
        rf_setup.bits.rf_dr_low = 1U;

        //g_nrf24_instance.transmit_receive_delay_us = 155;
    }
    else if(data_rate == NRF24_HAL_DATA_RATE_2MBPS) {
        rf_setup.bits.rf_dr_high = 1U;

        //g_nrf24_instance.transmit_receive_delay_us = 65;
    }

    nrf24_intl_write_register(RF_SETUP_REG, rf_setup.value);
}

nrf24_hal_datarate_t nrf24_hal_get_data_rate ()
{
    rf_setup_t  rf_setup_reg;

    rf_setup_reg.value = nrf24_intl_read_register(RF_SETUP_REG);

    if(rf_setup_reg.bits.rf_dr_low) {
        return NRF24_HAL_DATA_RATE_250KBPS;
    }
    else if(rf_setup_reg.bits.rf_dr_high) {
        return NRF24_HAL_DATA_RATE_2MBPS;
    }

    return NRF24_HAL_DATA_RATE_1MBPS;
}

void nrf24_hal_set_rf_output_power(nrf24_hal_rf_output_power_t rf_output_power)
{
    rf_setup_t  rf_setup_reg;

    rf_setup_reg.value = nrf24_intl_read_register(RF_SETUP_REG);
    rf_setup_reg.bits.rf_pwr = (uint8_t) rf_output_power;

    nrf24_intl_write_register(RF_SETUP_REG, rf_setup_reg.value);
}

/*
Clear one selected interrupt flag.
Use this function to clear one spesific interrupt flag.
Other interrupt flags are left unchanged.
*/
void nrf24_hal_clear_irq_flag(nrf24_hal_irq_source_t  int_source)
{
    SET_BIT(STATUS_REG, int_source);
}

/*Read then clears all interrupt flags. Use this function to get the interrupt flags and
clear them in the same operation. Reduced radio interface activity and speed optimized.
*/
uint8_t nrf24_hal_get_clear_irq_flags(void)
{
    uint8_t retval;

    retval = nrf24_intl_write_register(STATUS_REG, (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)));

    return (retval & (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)));
}

uint8_t nrf24_hal_clear_irq_flags_get_status(void)
{
    uint8_t retval;

    // When RF IRQ is cleared (when calling write_reg), pipe information is unreliable (read again with read_reg)
    retval = nrf24_intl_write_register(STATUS_REG, (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)));

    // Since it is serially shifted out from the above call we need to clear all the irq flags. Because the previous call is to clear the flags.
    retval &= (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    retval |= nrf24_intl_read_register(STATUS_REG) & 0x0F;

    return (retval);
}

uint8_t nrf24_hal_get_irq_flags(void)
{
    return (nrf24_intl_get_status_value() & (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)));
}

uint8_t nrf24_hal_get_rx_data_source(void)
{
    return ((nrf24_intl_get_status_value() >> 1) & 0x07);
}

uint8_t nrf24_hal_get_current_auto_retransmit_count(void)
{
    return (nrf24_intl_read_register(OBSERVE_TX_REG) & 0x0F);
}

void nrf24_hal_set_address_for_pipe(nrf24_hal_pipe_t    pipe_no,
                                    const uint8_t*      address)
{
    if(pipe_no != NRF24_HAL_PIPE_ALL) {
        uint8_t reg_addr = W_REGISTER + RX_PIPE_0_ADDR_REG + pipe_no;

        /*Pipe 0 & 1*/
        if((pipe_no < 2) || (pipe_no == NRF24_HAL_PIPE_TX)) {
            nrf24_intl_write_multibyte_register(reg_addr, address, nrf24_hal_get_address_width());
        }
        else {
            nrf24_intl_write_register(reg_addr, *address);
        }
    }
}

void nrf24_hal_set_transmit_address(const uint8_t*  address)
{
    nrf24_intl_write_multibyte_register(W_REGISTER + TX_ADDR_REG, address, nrf24_hal_get_address_width());
}

uint8_t nrf24_hal_get_rx_payload_width(uint8_t pipe_no)
{
    return (nrf24_intl_read_register(RX_PIPE_0_PAYLOAD_WIDTH_REG + pipe_no) & 0x3F);
}

/*
Set payload width for selected pipe. Use this function to set the number of bytes expected on a selected pipe
*/
void nrf24_hal_set_rx_payload_width(uint8_t pipe_no,
                                    uint8_t payload_width)
{
    nrf24_intl_write_register(RX_PIPE_0_PAYLOAD_WIDTH_REG + pipe_no, payload_width);
}

uint8_t nrf24_hal_is_tx_fifo_full(void)
{
    return is_bit_set(FIFO_STATUS_REG, TX_FULL);
}

uint8_t nrf24_hal_is_tx_fifo_empty(void)
{
    return is_bit_set(FIFO_STATUS_REG, TX_EMPTY);
}

uint8_t nrf24_hal_is_rx_fifo_full(void)
{
    return is_bit_set(FIFO_STATUS_REG, RX_FULL);
}

uint8_t nrf24_hal_is_rx_fifo_empty(void)
{
    return is_bit_set(FIFO_STATUS_REG, RX_EMPTY);
}

void nrf24_hal_set_dpl_for_pipe(nrf24_hal_pipe_t    pipe,
                                uint8_t             is_enable)
{
    if(pipe == NRF24_HAL_PIPE_ALL) {
        nrf24_intl_write_register(DYNAMIC_PAYLOAD_ENABLE_REG, (is_enable) ? 0x3F : 0x00);
    }
    else {
        if(is_enable) {
            set_bit(DYNAMIC_PAYLOAD_ENABLE_REG, pipe);
        }
        else {
            clear_bit(DYNAMIC_PAYLOAD_ENABLE_REG, pipe);
        }
    }
}

void nrf24_hal_set_dpl_feature(uint8_t is_enable)
{
    if(is_enable) {
        set_bit(FEATURE_REG, EN_DPL);
    }
    else {
        clear_bit(FEATURE_REG, EN_DPL);
    }
}

/*
If ACK packet payload is activated, ACK packets have dynamic payload lengths and the Dynamic Payload
Length feature should be enabled for pipe 0 on the PTX and PRX. This is to ensure that they receive the
ACK packets with payloads. If the ACK payload is more than 15 byte in 2Mbps mode the ARD must be
500uS or more, and if the ACK payload is more than 5 byte in 1Mbps mode the ARD must be 500uS or
more. In 250kbps mode (even when the payload is not in ACK) the ARD must be 500uS or more.*/
void nrf24_hal_set_ack_pkt_payload_feature(uint8_t is_enable)
{
    if(is_enable) {
        set_bit(FEATURE_REG, EN_ACK_PAY);

        nrf24_hal_set_dpl_feature(1);
        nrf24_hal_set_dpl_for_pipe(DPL_P0, 1);
    }
    else {
        clear_bit(FEATURE_REG, EN_ACK_PAY);
    }
}

uint8_t nrf24_hal_is_ack_pkt_payload_feature_enabled(void)
{
    return is_bit_set(FEATURE_REG, EN_ACK_PAY);
}

void nrf24_hal_set_no_ack_tx_payload_feature(uint8_t is_enable)
{
    if(is_enable) {
        set_bit(FEATURE_REG, EN_DYN_ACK);
    }
    else {
        clear_bit(FEATURE_REG, EN_DYN_ACK);
    }
}

#if 0
void nrf24_hal_reset()
{
    int i;

    nrf24_intl_write_register(CONFIG_REG, 0x08);
    nrf24_intl_write_register(ENABLE_AUTO_ACK_REG, 0x3F);
    nrf24_intl_write_register(ENABLE_RX_ADDR_REG, 0x02);
    nrf24_intl_write_register(SETUP_ADDRESS_WIDTH_REG, 0x03);
    nrf24_intl_write_register(SETUP_RETRANSMISSION_REG, 0x03);
    nrf24_intl_write_register(RF_CHANNEL_REG, 0x02);
    nrf24_intl_write_register(RF_SETUP_REG, 0x0E);
    nrf24_intl_write_register(STATUS_REG, 0x0E);
    nrf24_intl_write_register(OBSERVE_TX_REG, 0x00);

    for (i = 0; i < 6; i++)
    {
        nrf24_intl_write_register(RX_PIPE_0_PAYLOAD_WIDTH_REG + i, 0x00);
    }

    nrf24_intl_write_register(FIFO_STATUS_REG, 0x11);
    nrf24_intl_write_register(DYNAMIC_PAYLOAD_ENABLE_REG, 0x00);
    nrf24_intl_write_register(FEATURE_REG, 0x00);
}
#endif

void nrf24_hal_print_details(uint8_t print_reg_value)
{
    uint8_t i;
    uint8_t loop;
    uint8_t reg_value;
    uint8_t addr[5];
    uint8_t addr_width;

    reg_value = nrf24_intl_read_register(CONFIG_REG);
    NRF24_DEBUG_PRINTF(printf_P(PSTR("Power\t\t= %S\n"), pgm_read_word(&power_str_table_P[(reg_value >> 1) & 0x01]));)
    NRF24_DEBUG_PRINTF(printf_P(PSTR("Op Mode\t\t= %S\n"), pgm_read_word(&device_operating_mode_str_table_P[reg_value & 0x01]));)

    reg_value = nrf24_intl_read_register(ENABLE_AUTO_ACK_REG);

    NRF24_DEBUG_PRINTF(printf_P(PSTR("Auto Ack\t= Enabled["));)

    for (loop = 0; loop < 6; loop++)
    {
        if(((reg_value >> loop) & 0x01))
        {
            NRF24_DEBUG_PRINTF(printf_P(PSTR("%d,"), loop);)
        }
    }

    NRF24_DEBUG_PRINTF(printf_P(PSTR("]\n"));)

    reg_value = nrf24_intl_read_register(ENABLE_RX_ADDR_REG);

    NRF24_DEBUG_PRINTF(printf_P(PSTR("Enabled Pipes\t= Enabled["));)

    for (loop = 0; loop < 6; loop++)
    {
        if(((reg_value >> loop) & 0x01))
        {
            NRF24_DEBUG_PRINTF(printf_P(PSTR("%d,"), loop);)
        }
    }

    NRF24_DEBUG_PRINTF(printf_P(PSTR("]\n"));)

    addr_width = nrf24_hal_get_address_width();

    NRF24_DEBUG_PRINTF(printf_P(PSTR("Addr Width\t= %d Bytes\n"), addr_width);)

    reg_value = nrf24_intl_read_register(SETUP_RETRANSMISSION_REG);
    NRF24_DEBUG_PRINTF(printf_P(PSTR("Auto Re Delay\t= %d uSec\n"), ((reg_value >> 4) & 0x0F) * 250 + 250 );)

    NRF24_DEBUG_PRINTF(printf_P(PSTR("Auto Re Count\t= %d times\n"), (reg_value & 0x0F));)

    reg_value = nrf24_intl_read_register(RF_CHANNEL_REG);
    NRF24_DEBUG_PRINTF(printf_P(PSTR("RF Channel\t= %d\n"), reg_value);)

    NRF24_DEBUG_PRINTF(printf_P(PSTR("Data Rate\t= %S\n"), pgm_read_word(&data_rate_str_P[nrf24_hal_get_data_rate()]));)
    NRF24_DEBUG_PRINTF(printf_P(PSTR("CRC Len\t\t= %S\n"), pgm_read_word(&crc_length_str_P[nrf24_hal_get_crc_length()]));)

    reg_value = nrf24_intl_read_register(RF_SETUP_REG);
    NRF24_DEBUG_PRINTF(printf_P(PSTR("Out Power\t= %S\n"), pgm_read_word(&out_power_str_P[(reg_value >> 1) & 0x03]));)

    NRF24_DEBUG_PRINTF(printf_P(PSTR("Pipes PL Width\t= ["));)

    for (i = 0; i < 6; i++)
    {
        reg_value = nrf24_intl_read_register(RX_PIPE_0_PAYLOAD_WIDTH_REG+i);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("%d,"), reg_value);)
    }

    NRF24_DEBUG_PRINTF(printf_P(PSTR("]\n"));)

    reg_value = nrf24_intl_read_register(DYNAMIC_PAYLOAD_ENABLE_REG);

    NRF24_DEBUG_PRINTF(printf_P(PSTR("DPL Pipes\t= Enabled["));)

    for (loop = 0; loop < 6; loop++)
    {
        if(((reg_value >> loop) & 0x01))
        {
            NRF24_DEBUG_PRINTF(printf_P(PSTR("%d,"), loop);)
        }
    }

    NRF24_DEBUG_PRINTF(printf_P(PSTR("]\n"));)

    reg_value = nrf24_intl_read_register(FEATURE_REG);
    NRF24_DEBUG_PRINTF(printf_P(PSTR("Dyn Payload\t= %S\n"), pgm_read_word(&en_dis_str_table_P[((reg_value >> 2) & 0x01)]));)
    NRF24_DEBUG_PRINTF(printf_P(PSTR("Ack Payload\t= %S\n"), pgm_read_word(&en_dis_str_table_P[((reg_value >> 1) & 0x01)]));)
    
    NRF24_DEBUG_PRINTF(printf_P(PSTR("NoAck Cmd\t= %S\n"), pgm_read_word(&en_dis_str_table_P[(reg_value & 0x01)]));)

    nrf24_intl_read_multibyte_register(TX_ADDR_REG, &addr[0], addr_width);

    NRF24_DEBUG_PRINTF(printf_P(PSTR("Tx Addr\t\t= 0x"));)

    for (loop = 0; loop < addr_width - 1; loop++)
    {
        NRF24_DEBUG_PRINTF(printf_P(PSTR("%02X"), addr[addr_width - 2 - loop]);)
    }

    NRF24_DEBUG_PRINTF(printf_P(PSTR("\n"));)

    for (i = 0; i < 2; i++)
    {
        nrf24_intl_read_multibyte_register(RX_PIPE_0_ADDR_REG + i, &addr[0], addr_width);

        NRF24_DEBUG_PRINTF(printf_P(PSTR("PIPE %d Addr\t= 0x"), i);)

        for (loop = 0; loop < addr_width - 1; loop++)
        {
            NRF24_DEBUG_PRINTF(printf_P(PSTR("%02X"), addr[addr_width - 2 - loop]);)
        }
        NRF24_DEBUG_PRINTF(printf_P(PSTR("\n"));)
    }

    for (i = 2; i < 6; i++)
    {
        reg_value = nrf24_intl_read_register(RX_PIPE_0_ADDR_REG + i);
        addr[0] = reg_value;

        NRF24_DEBUG_PRINTF(printf_P(PSTR("PIPE %d Addr\t= 0x"), i);)

        for (loop = 0; loop < addr_width - 1; loop++)
        {
            NRF24_DEBUG_PRINTF(printf_P(PSTR("%02X"), addr[addr_width - 2 - loop]);)
        }
        NRF24_DEBUG_PRINTF(printf_P(PSTR("\n"));)
    }
    
    NRF24_DEBUG_PRINTF(printf_P(PSTR("\n"));)

    if (print_reg_value)
    {
        reg_value = nrf24_intl_read_register(CONFIG_REG);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("CONFIG\t\t0x%02x\n"), reg_value);)
        
        reg_value = nrf24_intl_read_register(ENABLE_AUTO_ACK_REG);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("EN_AA\t\t0x%02x\n"), reg_value);)
        
        reg_value = nrf24_intl_read_register(ENABLE_RX_ADDR_REG);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("EN_RXADDR\t0x%02x\n"), reg_value);)
        
        reg_value = nrf24_intl_read_register(SETUP_ADDRESS_WIDTH_REG);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("SETUP_AW\t0x%02x\n"), reg_value);)
        
        reg_value = nrf24_intl_read_register(SETUP_RETRANSMISSION_REG);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("SETUP_RETR\t0x%02x\n"), reg_value);)
        
        reg_value = nrf24_intl_read_register(RF_CHANNEL_REG);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("RF_CH\t\t0x%02x\n"), reg_value);)
        
        reg_value = nrf24_intl_read_register(RF_SETUP_REG);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("RF_SETUP\t0x%02x\n"), reg_value);)
        
        reg_value = nrf24_intl_read_register(STATUS_REG);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("STATUS\t\t0x%02x\n"), reg_value);)
        
        reg_value = nrf24_intl_read_register(OBSERVE_TX_REG);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("OBSER_TX\t0x%02x\n"), reg_value);)
        
        reg_value = nrf24_intl_read_register(FIFO_STATUS_REG);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("FIFO_STATUS\t0x%02x\n"), reg_value);)
        
        reg_value = nrf24_intl_read_register(DYNAMIC_PAYLOAD_ENABLE_REG);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("DYNPD\t\t0x%02x\n"), reg_value);)
        
        reg_value = nrf24_intl_read_register(FEATURE_REG);
        NRF24_DEBUG_PRINTF(printf_P(PSTR("FEATURE\t\t0x%02x\n"), reg_value);)
        
        NRF24_DEBUG_PRINTF(printf_P(PSTR("\n"));)
    }
}
