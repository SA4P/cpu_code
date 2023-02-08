/*
 * recv_task.c
 *
 *  Created on: Apr 2, 2022
 *      Author: SA4P Authors
 */
#include <s_recv_task.h>

static int test_cnt = 0;

typedef enum{
	STATE_GET_PAYLOAD_TYPE	= 0,
	STATE_GET_PAYLOAD_LEN	= 1,
	STATE_GET_PAYLOAD		= 2
} state_t;

//void task_receive_

// Checks for a byte if it is a valid payload type byte
bool_t s_check_payload_type(byte_t payload_type){

	switch (payload_type) {
		case SERVER_PAYLOAD_SIGNUP_REQ:
			return FALSE;
		case SERVER_PAYLOAD_SIGNUP_RESP:
			return TRUE;
		case SERVER_PAYLOAD_AUTH_REQ:
			return FALSE;
		case SERVER_PAYLOAD_AUTH_RESP:
			return TRUE;
		case SERVER_PAYLOAD_CONTROL:				// Note: Control not implemented
			return FALSE;
		default:									// Unknown payload type
			return FALSE;
			break;
	}
}

bool_t s_check_payload_len(uint16_t len, byte_t payload_type){

	switch (payload_type) {
		case SERVER_PAYLOAD_SIGNUP_REQ:
			// Should NEVER get in here, because if the payload type is PAYLOAD_SIGNUP_REQ, then we should not have even called rx_check_len!
			while(TRUE){
				__NOP();
			}
			return len == LEN_SERVER_PAYLOAD_SIGNUP_REQ;

		case SERVER_PAYLOAD_SIGNUP_RESP:
			return len == LEN_SERVER_PAYLOAD_SIGNUP_RESP;

		case SERVER_PAYLOAD_AUTH_REQ:
			// Should NEVER get in here, because if the payload type is SERVER_PAYLOAD_AUTH_REQ, then we should not have even called rx_check_len!
			while(TRUE){
				__NOP();
			}
			return len == LEN_SERVER_PAYLOAD_AUTH_REQ;

		case SERVER_PAYLOAD_AUTH_RESP:
			return len == LEN_SERVER_PAYLOAD_AUTH_RESP;

		case SERVER_PAYLOAD_CONTROL:				// Note: Control not implemented
			// Should NEVER get in here, because SERVER_PAYLOAD_CONTROL is not implemented!
			while(TRUE){
				__NOP();
			}
			return len == LEN_SERVER_PAYLOAD_CONTROL;

		default:										// Unknown payload type
			return FALSE;

	}

}

void s_task_receive(msg_t* msg_state){
	static state_t state = STATE_GET_PAYLOAD_TYPE;

	bool_t valid;														// Set to 1 if value is in permissible range

	if(state == STATE_GET_PAYLOAD_TYPE){

		byte_t b;
		if(rb_consume_one(&lp1_rx_rbuf, &b)){
			valid = s_check_payload_type(b);
			if(valid){
				msg_state->payload_type = b;
				state = STATE_GET_PAYLOAD_LEN;
			}
			else{																				// Case: Bad/unknown payload type
				// TODO: Somehow notify for resynchronization and debug purposes!
				__NOP();
			}
		}
		else{
			return;
		}
	}

	if(state == STATE_GET_PAYLOAD_LEN){
		uint16_t len;

		if(rb_consume(&lp1_rx_rbuf, &len, 2)){														// Try to consume 2 bytes from receive buffer
			valid = s_check_payload_len(len, msg_state->payload_type);
			if(valid){																			// Check whether received length is valid
				msg_state->payload_len = len;
				state = STATE_GET_PAYLOAD;
			}
			else{																				// Case: Received length invalid ==> Discard and start anew
				// TODO: Somehow notify for resynchronization and debug purposes!
				state = STATE_GET_PAYLOAD_TYPE;
			}
		}
	}

	if(state == STATE_GET_PAYLOAD){
		uint16_t len = msg_state->payload_len;
		if(rb_consume(&lp1_rx_rbuf, msg_state->p_payload, len)){
			state = STATE_GET_PAYLOAD_TYPE;
			s_flags.rx_done = TRUE;
			test_cnt++;
		}
	}
}
