/*
 * helper.h
 *
 *  Created on: May 24, 2022
 *      Author: SA4P Authors
 */

#ifndef INC_HELPER_H_
#define INC_HELPER_H_

#include "main.h"

void rcc_enable_gpiog();
void rcc_enable_gpiod();
void rcc_enable_gpioc();
void rcc_enable_gpioe();
void rcc_enable_gpiof();

void rcc_enable_lpuart();
void rcc_enable_usart2();
void rcc_set_vddio2();

void gpio_init_uart(USART_TypeDef * uart, bool_t to_usb);
void gpio_init_button();
void gpio_init_port_as_output();

void usart_init_cr1(USART_TypeDef * uart);
void usart_init_cr2(USART_TypeDef * uart);
void usart_init_cr3(USART_TypeDef * uart);
void usart_init_baudrate(USART_TypeDef * uart, uint32_t baudrate);

void usart_enable(USART_TypeDef * uart);
void usart_enable_receive(USART_TypeDef * uart);
void usart_enable_transmit(USART_TypeDef * uart);
void usart_set_TXEIE(USART_TypeDef * uart);
void usart_set_RXNEIE(USART_TypeDef * uart);

void   rb_init(ringbuf_t * rb, char init_val);
bool_t rb_consume_one(ringbuf_t* rb, byte_t* obyte);
bool_t rb_produce_one(ringbuf_t* rb, byte_t ibyte);

int16_t rb_consume_all(ringbuf_t* rb, byte_t* obuf);

bool_t rb_consume(ringbuf_t* rb, byte_t * obuf, int16_t num_items);
bool_t rb_produce(ringbuf_t* rb, byte_t* ibuf, int16_t num_items);


void timer_init(TIM_TypeDef* t, uint16_t presc);
void timer_start(TIM_TypeDef* t, uint16_t timeout);
bool_t s_msg_build(byte_t msg_type, req_t* req_state, msg_t* i_msg, send_msg_t* o_msg);
bool_t gw_msg_build(byte_t msg_type, req_t* req_state, byte_t* i_buf, send_msg_t* o_msg);
bool_t process_bytes();

// Generic helper functions
int min(int a, int b);
int16_t mod(int16_t x, int16_t m);



#endif /* INC_HELPER_H_ */
