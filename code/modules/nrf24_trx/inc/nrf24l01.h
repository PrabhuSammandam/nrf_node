
/* Memory Map */

/* nRF24L01+ Command Definitions */
#define R_REGISTER			0x00U 	// Read command and status registers.

#define W_REGISTER          0x20U 	// Write command and status registers. Executable in power down or standby modes only.

#define R_RX_PAYLOAD        0x61U 	// Read RX-payload: 1 – 32 bytes. A read operation always starts at byte 0.
									// Payload is deleted from FIFO after it is read. Used in RX mode.

#define W_TX_PAYLOAD        0xA0U	// Write TX-payload: 1 – 32 bytes. A write operation
									// always starts at byte 0 used in TX payload.
									
#define FLUSH_TX            0xE1U	// Flush TX FIFO, used in TX mode

#define FLUSH_RX            0xE2U	// Flush RX FIFO, used in RX mode Should not be executed during transmission of
									// acknowledge, that is, acknowledge package will not be completed.

#define REUSE_TX_PL         0xE3U	// Used for a PTX device Reuse last transmitted payload. TX payload reuse is active until
									// W_TX_PAYLOAD or FLUSH TX is executed. TX payload reuse must not be activated or
									// deactivated during package transmission.

#define ACTIVATE            0x50U	// 

#define R_RX_PL_WID         0x60U	// Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO.
									// Note: 1. Flush RX FIFO if the read value is larger than 32 bytes.
									// 2. The bits in the FEATURE register EN_DPL have to be set.

#define W_ACK_PAYLOAD       0xA8U	// Used in RX mode. Write Payload to be transmitted together with
									// ACK packet on PIPE PPP. (PPP valid in the range from 000 to 101).
									// Maximum three ACK packet payloads can be pending. Payloads with
									// same PPP are handled using first in - first out principle.
									// Write payload: 1– 32 bytes. A write operation always starts at byte 0.
									// Note: The bits in the FEATURE register EN_ACK_PAY have to be set.

#define W_TX_PAYLOAD_NOACK  0xB0U	// Used in TX mode. Disables AUTOACK on this specific packet.
									// Note: The bits in the FEATURE register EN_DYN_ACK have to be set.

#define NOP                 0xFFU	// No Operation. Might be used to read the STATUS register

/* nRF24L01+  Register Definitions * */
#define CONFIG_REG                  0x00	// config register
#define ENABLE_AUTO_ACK_REG         0x01	// Enable ‘Auto Acknowledgment’ Function. Disable this functionality to be compatible with nRF2401
#define ENABLE_RX_ADDR_REG          0x02	// enable RX addresses register
#define SETUP_ADDRESS_WIDTH_REG     0x03	// setup of address width register
#define SETUP_RETRANSMISSION_REG    0x04	// setup of automatic retransmission register
#define RF_CHANNEL_REG              0x05	// RF channel register
#define RF_SETUP_REG                0x06	// RF setup register
#define STATUS_REG                  0x07	// status register
#define OBSERVE_TX_REG              0x08	// transmit observe register
#define RECEIVED_POWER_DETECTOR_REG 0x09	// carrier detect register
#define RX_PIPE_0_ADDR_REG          0x0A	// receive address data pipe0
#define RX_PIPE_1_ADDR_REG          0x0B	// receive address data pipe1
#define RX_PIPE_2_ADDR_REG          0x0C	// receive address data pipe2
#define RX_PIPE_3_ADDR_REG          0x0D	// receive address data pipe3
#define RX_PIPE_4_ADDR_REG          0x0E	// receive address data pipe4
#define RX_PIPE_5_ADDR_REG          0x0F	// receive address data pipe5
#define TX_ADDR_REG                 0x10	// transmit address
#define RX_PIPE_0_PAYLOAD_WIDTH_REG 0x11	// # of bytes in rx payload for pipe0
#define RX_PIPE_1_PAYLOAD_WIDTH_REG 0x12	// # of bytes in rx payload for pipe1
#define RX_PIPE_2_PAYLOAD_WIDTH_REG 0x13	// # of bytes in rx payload for pipe2
#define RX_PIPE_3_PAYLOAD_WIDTH_REG 0x14	// # of bytes in rx payload for pipe3
#define RX_PIPE_4_PAYLOAD_WIDTH_REG 0x15	// # of bytes in rx payload for pipe4
#define RX_PIPE_5_PAYLOAD_WIDTH_REG 0x16	// # of bytes in rx payload for pipe5
#define FIFO_STATUS_REG             0x17	// FIFO status register
#define DYNAMIC_PAYLOAD_ENABLE_REG  0x1C	// Dynamic payload setup
#define FEATURE_REG                 0x1D	// Exclusive feature setup

/* CONFIG 0x00 - Configuration Register */
/** @name CONFIG register bit definitions */
//@{
#define MASK_RX_DR          6	// Mask interrupt caused by RX_DR
								// 1: Interrupt not reflected on the IRQ pin
								// 0: Reflect RX_DR as active low interrupt on the IRQ pin
								
#define MASK_TX_DS          5	// Mask interrupt caused by TX_DS
								// 1: Interrupt not reflected on the IRQ pin
								// 0: Reflect TX_DS as active low interrupt on the IRQ pin
								
#define MASK_MAX_RT         4	// Mask interrupt caused by MAX_RT
								// 1: Interrupt not reflected on the IRQ pin
								// 0: Reflect MAX_RT as active low interrupt on the IRQ pin
								
#define ENABLE_CRC_BIT      3	// Enable CRC. Forced high if one of the bits in the EN_AA is high

#define CRC_ENCODING_BIT    2	// CRC encoding scheme. 0 => 1 byte, 1 => 2 bytes

#define POWER_UP_BIT        1	// 1 => POWER UP, 0 => POWER DOWN

#define PRIMARY_RX_BIT      0	// RX/TX control. 1 => PRX, 0 => PTX
//@}

/* EN_AA 0x01 - Enable ‘Auto Acknowledgment’ Function Register */
/** @name EN_AA register bit definitions */
//@{
#define ENABLE_AUTO_ACK_PIPE_5_BIT  5	// Enable auto acknowledgement data pipe 5
#define ENABLE_AUTO_ACK_PIPE_4_BIT  4	// Enable auto acknowledgement data pipe 4
#define ENABLE_AUTO_ACK_PIPE_3_BIT  3	// Enable auto acknowledgement data pipe 3
#define ENABLE_AUTO_ACK_PIPE_2_BIT  2	// Enable auto acknowledgement data pipe 2
#define ENABLE_AUTO_ACK_PIPE_1_BIT  1	// Enable auto acknowledgement data pipe 1
#define ENABLE_AUTO_ACK_PIPE_0_BIT  0	// Enable auto acknowledgement data pipe 0

//@}

/* EN_RXADDR 0x02 - Enabled RX Addresses Register */
/** @name EN_RXADDR register bit definitions */
//@{
#define ENABLE_RX_PIPE_5_BIT    5	// Enable data pipe 5.
#define ENABLE_RX_PIPE_4_BIT    4	// Enable data pipe 4.
#define ENABLE_RX_PIPE_3_BIT    3	// Enable data pipe 3.
#define ENABLE_RX_PIPE_2_BIT    2	// Enable data pipe 2.
#define ENABLE_RX_PIPE_1_BIT    1	// Enable data pipe 1.
#define ENABLE_RX_PIPE_0_BIT    0	// Enable data pipe 0.
//@}

/* SETUP_RETR 0x04 - Setup of Automatic Retransmission Register */
/** @name SETUP_RETR register bit definitions */
//@{
#define ARD         4	// Auto Retransmit Delay
						// ‘0000’ – Wait 250uS
						// ‘0001’ – Wait 500uS
						// ‘0010’ – Wait 750uS
						// ……..
						// ‘1111’ – Wait 4000uS
						// (Delay defined from end of transmission to start of next transmission)b
						
#define ARC         0	// Auto Retransmit Count
						// ‘0000’ –Re-Transmit disabled
						// ‘0001’ – Up to 1 Re-Transmit on fail of AA
						// ……
						// ‘1111’ – Up to 15 Re-Transmit on fail of AA
//@}

/* RF_SETUP 0x06 - RF Setup Register Register */
/** @name RF_SETUP register bit definitions */
//@{
#define CONT_WAVE	7	// Enables continuous carrier transmit when high.
#define RF_DR_LOW	5	// Set RF Data Rate to 250kbps. See RF_DR_HIGH for encoding.
#define PLL_LOCK	4	// Force PLL lock signal. Only used in test

#define RF_DR_HIGH	3	// Select between the high speed data rates. This bit is don’t care if RF_DR_LOW is set.
						// Encoding:
						// [RF_DR_LOW, RF_DR_HIGH]:
						// ‘00’ – 1Mbps
						// ‘01’ – 2Mbps
						// ‘10’ – 250kbps
						// ‘11’ – Reserved

#define RF_PWR		2	// Set RF output power in TX mode
						// '00' – -18dBm
						// '01' – -12dBm
						// '10' – -6dBm
						// '11' – 0dBm
//@}

/* STATUS 0x07 - Status register */
/** @name STATUS register bit definitions */
//@{
#define RX_DR       6	// Data Ready RX FIFO interrupt. Asserted when new data arrives RX FIFO. Write 1 to clear bit.

#define TX_DS       5	// Data Sent TX FIFO interrupt. Asserted when packet transmitted on TX.
						// If AUTO_ACK is activated, this bit is set high only when ACK is received.
						// Write 1 to clear bit.

#define MAX_RT      4	// Maximum number of TX retransmits interrupt
						// Write 1 to clear bit. If MAX_RT is asserted it must
						// be cleared to enable further communication.

#define RX_P_NO     1	// Data pipe number for the payload available for reading from RX_FIFO
						// 000-101: Data Pipe Number
						// 110: Not Used
						// 111: RX FIFO Empty

#define TX_FULL     0	// TX FIFO full flag. 1=> TX FIFO full. 0=> Available locations in TX FIFO.
//@}


/* OBSERVE_TX 0x08 - Transmit observe register */
/** @name OBSERVE_TX register bit definitions */
//@{
#define PLOS_CNT    4	// Count lost packets. The counter is overflow protected
						// to 15, and discontinues at max until reset.
						// The counter is reset by writing to RF_CH.
#define ARC_CNT     0   // Count retransmitted packets. The counter is reset
						// when transmission of a new packet starts
//@}

/*$off*/
/* FIFO_STATUS 0x17 - FIFO Status Register */
/** @name FIFO_STATUS register bit definitions */
//@{
#define TX_REUSE    6						// Used for a PTX device
                                            // Pulse the rfce high for at least 10us to Reuse last
                                            // transmitted payload. TX payload reuse is active
                                            // until W_TX_PAYLOAD or FLUSH TX is executed.
                                            // TX_REUSE is set by the SPI command
                                            // REUSE_TX_PL, and is reset by the SPI commands
                                            // W_TX_PAYLOAD or FLUSH TX
/*$on*/
#define TX_FIFO_FULL    5                   // TX FIFO full flag. 1=> TX FIFO full. 0=> Available locations in TX FIFO.
#define TX_EMPTY        4                   // TX FIFO empty flag. 1=> TX FIFO empty. 0=> Data in TX FIFO.
#define RX_FULL         1                   // RX FIFO full flag. 1=> RX FIFO full. 0=> Available locations in RX FIFO.
#define RX_EMPTY        0                   // RX FIFO empty flag. 1=> RX FIFO empty. 0=> Data in RX FIFO.

//@}

/* DYNPD 0x1C - Enable dynamic payload length */
/** @name DYNPD register bit definitions */
//@{
#define DPL_P5  5       // Enable dynamic payload length data pipe 5. (Requires EN_DPL and ENAA_P5)
#define DPL_P4  4       // Enable dynamic payload length data pipe 4. (Requires EN_DPL and ENAA_P4)
#define DPL_P3  3       // Enable dynamic payload length data pipe 3. (Requires EN_DPL and ENAA_P3)
#define DPL_P2  2       // Enable dynamic payload length data pipe 2. (Requires EN_DPL and ENAA_P2)
#define DPL_P1  1       // Enable dynamic payload length data pipe 1. (Requires EN_DPL and ENAA_P1)
#define DPL_P0  0       // Enable dynamic payload length data pipe 0. (Requires EN_DPL and ENAA_P0)
//@}

/* FEATURE 0x1D - Feature Register */
/** @name FEATURE register bit definitions */
//@{
#define EN_DPL      2   // Enables Dynamic Payload Length
#define EN_ACK_PAY  1   // Enables Payload with ACK
#define EN_DYN_ACK  0   // Enables the W_TX_PAYLOAD_NOACK command


/*
Important points for the device working

1. Set RX_ADDR_P0 equal to TX_ADDR address to handle automatic acknowledge if this is a 
	PTX device with Enhanced ShockBurst™ enabled.

2. Set Auto Retry Count to 0 to disable the AutoRetry feature.

3. When a MAX_RT interrupt occurs the TX payload will not be removed from the TX FIFO.
   If the packet is to be discarded this must be done manually by flushing the TX FIFO.
   Alternatively, CE_PULSE() can be called re-starting transmission of the payload.
   (Will only be possible after the radio irq flags are cleared)

*/
