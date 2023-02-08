/*
 * send_task.c
 *
 *  Created on: Apr 4, 2022
 *      Author: SA4P Authors
 */

#include "send_task.h"

static bool_t profiling_dummy_msg = TRUE;

bool_t s_task_send(send_msg_t* o_msg){

	// Flag already checked in main loop, just double check. If it is for whatever reason not set, we have an error because we should only enter task_send_s if we have data to send to the server, i.e. if s_flags.tx_rdy == TRUE!
	if(!s_flags.tx_rdy){
		while(TRUE){
			__NOP();
		}
	}

	// Set flag to false
	s_flags.tx_rdy = FALSE;

	if(!rb_produce(&lp1_tx_rbuf, o_msg->msg_buf, o_msg->len)){
		return FALSE;
	}

	// Profiling: Add a counter which tells the ISR what profiling stage it is in
	if(o_msg->msg_buf[0] == SERVER_PAYLOAD_AUTH_REQ){
		if(profiling_dummy_msg){
			profiling_dummy_msg = FALSE;
		}else{
			profiling_ctr_lpuart = 4;
		}
	}

	// Set Server-UART (LPUART1) to send out message
	usart_set_TXEIE(s_uart);

	return TRUE;
}

bool_t gw_task_send(send_msg_t* o_msg){

	// Flag already checked in main loop, just double check. If it is for whatever reason not set, we have an error because we should only enter task_send_gw if we have data to send to the gateway, i.e. if gw_flags.tx_rdy == TRUE!
	if(!gw_flags.tx_rdy){
		while(TRUE){
			__NOP();
		}
	}

	// Set flag to false
	gw_flags.tx_rdy = FALSE;

	if(!rb_produce(&us2_tx_rbuf, o_msg->msg_buf, o_msg->len)){
		return FALSE;
	}

	// Profiling: Add a counter which tells the ISR what profiling stage it is in
	if(o_msg->msg_buf[0] == GATEWAY_PAYLOAD_REQUEST){
		profiling_ctr_usart2 = 3;
	}else if(o_msg->msg_buf[0] == GATEWAY_PAYLOAD_RESPONSE){
		profiling_ctr_usart2 = 5;
	}

	// Set Gateway-USART (USART2) to send out message
	usart_set_TXEIE(gw_usart);

	return TRUE;
}

//bool_t task_send(conns_t conn_selector, send_msg_t* o_msg){
//
//	// (1) Select which connection we should send out of based on
//	ringbuf_t* p_tx_rbuf;
//	if(conn_selector == CONN_SERVER){
//		p_tx_rbuf = &lp1_tx_rbuf;
//	}
//	else if(conn_selector = CONN_GATEWAY){
//		p_tx_rbuf = &us2_tx_rbuf;
//	}
//	else{
//		while(TRUE){
//			__NOP();
//		}
//	}
//
//	// Flag already checked in main loop, just double check. If it is for whatever reason not set, we have an error because we should only enter task_send if flag_send_message == TRUE!
//	if(!flag_send_message){
//		while(TRUE){
//			__NOP();
//		}
//	}
//
//	// Set flag to false
//	flag_send_message = FALSE;
//
//	if(!rb_produce(p_tx_rbuf, o_msg->msg_buf, o_msg->len)){
//		return FALSE;
//	}
//
//	// Set UART to send out message
//	usart_set_TXEIE(lpuart);
//
//	return TRUE;
//}
