/*
 * gpio.h
 *
 * Created: 3/30/2016 12:23:25 PM
 *  Author: psammand
 */ 


#ifndef GPIO_H_
#define GPIO_H_

/************************************************************************/
/*  PIN direction setting api's                                                                     */
/************************************************************************/
#define gpio_set_pin_input(x) do { io_port_set_pin_input(x); } while(0);
#define gpio_set_pin_output(x) do { io_port_set_pin_output(x); } while(0);
#define gpio_is_pin_output(x) do { io_port_is_pin_output(x); } while(0);
#define gpio_is_pin_input(x) do { io_port_is_pin_input(x); } while(0);

/************************************************************************/
/* OUT  PIN manipulation api's                                                                     */
/************************************************************************/
#define gpio_set_pin_high(x) do { io_port_set_pin_high(x); } while(0);
#define gpio_set_pin_low(x) do { io_port_set_pin_low(x); } while(0);
#define gpio_toggle_pin(x) do { io_port_toggle_pin(x); } while(0);
#define gpio_set_pin(_PIN_,_VAL_) do { io_port_set_pin(_PIN_,_VAL_); } while(0);

/************************************************************************/
/*  IN PIN manipulation api's                                                                     */
/************************************************************************/
#define gpio_is_pin_high(x) do { io_port_is_pin_high(x); } while(0);
#define gpio_is_pin_low(x) do { io_port_is_pin_low(x); } while(0);

#endif /* GPIO_H_ */