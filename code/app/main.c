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
#include "utils/compiler.h"
#include "nrf24_hal.h"
#include "nrf24l01.h"
#include "nrf24_port.h"
#include "nrf24_params.h"
#include "gpio/gpio.h"

#define TRANSMITTER
#define RECEIVER

#define IS_BIT_CLEAR(_V_, _B_)  ((_V_ & (_BV(_B_))) == 0)
#define IS_BIT_SET(_V_, _B_)    ((_V_ & (_BV(_B_))) != 0)

static int  uart_putchar(char c, FILE* stream);
static void init_debug_prints(void);
static void run_as_transmitter(void);
static void run_as_receiver(void);

#define LED_GREEN       IOPORT_CREATE_PIN(PORT_D, 3)
#define RELAY_PIN       IOPORT_CREATE_PIN(PORT_D, 4)

#define SWITCH_ON_PIN   IOPORT_CREATE_PIN(PORT_D, 4)
#define SWITCH_OFF_PIN  IOPORT_CREATE_PIN(PORT_D, 5)

#define MOTION_PIN  IOPORT_CREATE_PIN(PORT_D, 5)

#define UART_BAUD_RATE  115200

static FILE uartout;

void board_init(void)
{
    timer_init();
    uart_init(UART_BAUD_RATE);
    spi_init();
    init_debug_prints();
    cpu_irq_enable();
    nrf24_link_init();

    gpio_set_pin_output(LED_GREEN);
}

uint8_t get_key(void)
{
    if(io_port_is_pin_low(SWITCH_OFF_PIN)) {
        delay_ms(25);

        if(io_port_is_pin_low(SWITCH_OFF_PIN)) {
            return 0;
        }
    }

    if(io_port_is_pin_low(SWITCH_ON_PIN)) {
        delay_ms(25);

        if(io_port_is_pin_low(SWITCH_ON_PIN)) {
            return 1;
        }
    }

    return 0xFF;
}

int main(void)
{

    board_init();

#ifdef TRANSMITTER
    run_as_transmitter();

#else
    run_as_receiver();
#endif
}

static void run_as_transmitter(void)
{
    uint8_t led_display_count = 0;
    uint8_t count = 0;
    uint8_t status;

    gpio_set_pin_input(SWITCH_ON_PIN);
    gpio_set_pin_input(SWITCH_OFF_PIN);

	/* enable internal pull up*/
    gpio_set_pin_high(SWITCH_ON_PIN);
    gpio_set_pin_high(SWITCH_OFF_PIN);

    nrf24_link_set_pan_id(0x0001);
    nrf24_link_set_nwk_id(0x0001);

    count = 0xFF;
    nrf24_hal_print_details(1);

    while(1)
    {
#if 0
        if(uart_available()) {
            uint8_t ch = uart_getc() & 0xFF;

            if(ch == '0') {
                count = 0;
                printf_P(PSTR("relay off\n"));
            }
            else if(ch == '1') {
                count = 1;
                printf_P(PSTR("relay on\n"));
            }
        }
#endif

        count = get_key();

        if(count != 0xFF) {
            uint8_t success = nrf24_link_tx_data(0x0001, 0x0002, &count, sizeof(uint8_t), NRF24_LL_UNICAST, NRF24_LL_NO_TIMEOUT);

            if(!success) {
                printf_P(PSTR("ds, count %d\n"), count);
            }

            count = 0xFF;
        }

        delay_ms(1);

        led_display_count++;

        if(led_display_count > 200) {
            led_display_count = 0;
            gpio_toggle_pin(LED_GREEN);
        }
    }
}

static void run_as_receiver(void)
{
    uint8_t led_display_count = 0;
    uint8_t count = 0;
    uint8_t status;

    gpio_set_pin_output(RELAY_PIN);
    gpio_set_pin_low(RELAY_PIN);

    nrf24_link_set_pan_id(0x0001);
    nrf24_link_set_nwk_id(0x0002);

    nrf24_link_rx_start();

    //CE_HIGH();
    nrf24_hal_print_details(1);

    while(1) {
        if(!nrf24_hal_is_rx_fifo_empty()) {
            nrf24_write_register(STATUS_REG, (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)));

            //nrf24_hal_flush_rx_fifo();
            nrf24_hal_read_rx_payload(&count);
            printf_P(PSTR("data received %d\n"), count);

            if(count == 0) {
                gpio_set_pin_low(RELAY_PIN);
            }
            else if(count == 1) {
                gpio_set_pin_high(RELAY_PIN);
            }
        }

#if 0
        delay_ms(1);
        led_display_count++;

        if(led_display_count > 200) {
            led_display_count = 0;
            gpio_toggle_pin(LED_GREEN);
        }
#endif

        //gpio_toggle_pin(RELAY_PIN);
    }
}

static int uart_putchar(char    c,
                        FILE*   stream)
{
    uart_putc(c);
    return 0;
}

static void init_debug_prints(void)
{
    fdev_setup_stream(&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
    stdout = &uartout;
}
