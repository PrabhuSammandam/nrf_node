/*
* nrf_rfd_node_v1.c
*
* Created: 19-03-2016 18:56:35
* Author : prabhu
*/
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "ioport/io_port.h"
#include "serial/uart.h"
#include "tick_timer/tick_timer.h"
#include "nrf24_hal.h"
#include "nrf24_link_layer.h"
#include "spi/spi.h"
#include "console/console.h"
#include "utils/interrupt.h"
#include "delay/delay_cycle.h"

static int uart_putchar (char c, FILE *stream);
static void init_debug_prints();

#define LED_RELAY          IOPORT_CREATE_PIN(PORT_C, 5)
#define LED_YELLOW          IOPORT_CREATE_PIN(PORT_C, 3)
#define LED_GREEN           IOPORT_CREATE_PIN(PORT_C, 2)
#define LED_DIGITAL_13           IOPORT_CREATE_PIN(PORT_B, 5)
#define LED_ARDUINO     IOPORT_CREATE_PIN(PORT_B, 5)


uint8_t count = 0;

#define UART_BAUD_RATE  115200

static FILE uartout;

uint8_t rx_buffer[32];
uint8_t enabled = 0xFF;
uint8_t dst_addr[5] = { 0x01, 0x00, 0xFF, 0xFF, 0xE7 };
uint8_t node_addr[5] = {0x01, 0x00, 0xFF,  0xFF, 0xE7 };

int main()
{
    timer_init();
    uart_init(UART_BAUD_RATE);
    spi_init();

    init_debug_prints();

    cpu_irq_enable();

    io_port_set_pin_output(LED_GREEN);
    io_port_set_pin_output(LED_YELLOW);
    io_port_set_pin_output(LED_DIGITAL_13);
    io_port_set_pin_output(LED_RELAY);

    nrf24_link_init();
    nrf24_link_set_pan_id(0x0010);
    nrf24_link_set_nwk_id(0x0001);

    while(1)
    {
        nrf24_hal_print_details(1);
        io_port_toggle_pin(LED_GREEN);
        io_port_toggle_pin(LED_YELLOW);
        //io_port_toggle_pin(LED_DIGITAL_13);
        printf_P(PSTR("Hello World\n"));

        delay_ms(1000);

        if(count++ > 15)
        {
            count = 0;
            //io_port_toggle_pin(LED_RELAY);
        }
    }
}

static int uart_putchar (char c, FILE *stream)
{
    uart_putc(c);
    return 0 ;
}

static void init_debug_prints()
{
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &uartout ;
}
#if 0
int main()
{
    uint8_t value = 0;

    timer_init();
    uart_init(UART_BAUD_RATE);
    spi_init();
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &uartout ;

    cpu_irq_enable();

    delay_ms(5000);

    nrf24_link_init();
    nrf24_link_set_pan_id(0x0010);
    nrf24_link_set_nwk_id(0x0001);

    //nrf24_hal_print_details(1);

    //value = nrf24_link_tx_data(0x0010, 0x0002, rx_buffer, 1, NRF24_LL_UNICAST, NRF24_LL_NO_TIMEOUT);
    //printf_P(PSTR("Transmit status %d\n"), value);

    while(1)
    {
        delay_ms(5000);
    }
}
#endif
#if 0
int main(void)
{
    uint8_t rx_length;
    uint8_t rx_pipe;

    timer_init();
    uart_init(UART_BAUD_RATE);
    spi_init();

    cpu_irq_enable();

    nrf24_link_init();

    while(1) {
        if(enabled == 1) {
            //uint32_t    time_millis = timer_millis();
            if(!nrf24_link_rx_fifo_read(&rx_buffer[0], &rx_length, &rx_pipe)) {
                uart_puts_P("Received Data in Node 1\n");
            }
            else
            {
                delay_ms(1000);
                uart_puts_P("No Data received in Node 1\n");
            }
        }

        if(enabled == 2) {
            uint32_t    time_millis = timer_millis();
            uint8_t     status;

            //if(!nrf24_link_rx_fifo_read(&rx_buffer[0], &rx_length, &rx_pipe)) {
            //    uart_puts_P("Received Data in Node 2\n");
            //}
            dst_addr[0] = 0x01;

            uart_puts_P("sending data\n");
            status = nrf24_link_tx_data(&dst_addr[0], (const uint8_t*) &time_millis, sizeof(uint32_t), NRF24_LL_UNICAST, NRF24_LL_NO_TIMEOUT);

            //uart_puts_P("getting status\n");
            //status = nrf24_link_block_get_tx_status();

            //uart_puts_P("starting receiver\n");
            //nrf24_link_rx_start();

            if(status) {
                uart_puts_P("Failed to send to node 2\n");
            }
            else
            {
                uart_puts_P("sent success\n");
            }
            delay_ms(1000);
        }

        if(uart_available()) {
            uint8_t ch = uart_getc() & 0xFF;

            if(ch == '1') {
                enabled = 1;
                dst_addr[0] = 0x01;

                nrf24_link_set_node_address(&dst_addr[0]);
                nrf24_link_rx_start();

                uart_puts_P("Enabled Node 1\n");
            }
            else if(ch == '2') {
                enabled = 2;
                dst_addr[0] = 0x02;

                nrf24_link_set_node_address(&dst_addr[0]);
                nrf24_link_rx_start();

                uart_puts_P("Enabled Node 2\n");
            }
        }
    }
}
#endif

#if 0
int main(void)
{
    #if 0
    io_port_set_pin_output(LED_ARDUINO);

    while(1) {
        io_port_toggle_pin(LED_ARDUINO);
        _delay_ms(100);
    }
    #endif

    #if 1
    timer_init();
    uart_init( /*UART_BAUD_SELECT(250000, F_CPU)*/ 250000);

    io_port_set_pin_output(LED_1);

    sei();

    uint32_t    milli_last = timer_millis();

    while(1) {
        _delay_ms(990);
        io_port_toggle_pin(LED_1);

        uart_puts_P("Hello World");
        milli_last = timer_millis();

        //console_print_long(milli_last);
        uart_putc('\n');

        while(uart_available() != 0) {
            uint8_t received_byte = (uint8_t) uart_getc();

            uart_putc(received_byte);

            //if(received_byte == 'a')
            //{
            //	uart_puts("received byte");
            //}
        }

        //uart_puts_P("Hello World\n");
    }
    #endif

    #if 0
    uint8_t         tx_status;
    uint8_t         pipe;
    uint8_t         length;
    uint8_t         buffer[NRF24_LL_MAX_FW_PAYLOAD_LENGTH];

    const uint8_t   node_address[NRF24_LL_ADDRESS_WIDTH] = { 0xe7, 0xff, 0xff, 0x00, 0x01 };
    const uint8_t   dst_address[NRF24_LL_ADDRESS_WIDTH] = { 0xe7, 0xff, 0xff, 0x00, 0x02 };

    timer_init();
    uart_init(UART_BAUD_SELECT(250000, F_CPU));
    spi_init();

    sei();

    nrf24_link_init();

    nrf24_link_set_node_address(&node_address[0]);

    nrf24_link_rx_start();

    PHY_Init();

    while(1) {
        if(nrf24_link_rx_fifo_read(&buffer[0], &length, &pipe)) {
        }

        timer_delay_ms(1000);

        *((uint32_t*) &buffer[0]) = timer_millis();

        tx_status = nrf24_link_tx_data(&dst_address[0], &buffer[0], sizeof(uint32_t), NRF24_LL_UNICAST, NRF24_LL_NO_TIMEOUT);
    }
    #endif
}
#endif
