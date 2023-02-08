/*
 * recv_task.c
 *
 *  Created on: Apr 2, 2022
 *      Author: SA4P Authors
 */
#include <gw_recv_task.h>

static int test_cnt = 0;

typedef enum{
	STATE_GET_PAYLOAD_TYPE	= 0,
	STATE_GET_PAYLOAD_LEN	= 1,
	STATE_GET_PAYLOAD		= 2
} state_t;

//void task_receive_

// Checks for a byte if it is a valid payload type byte
bool_t gw_check_payload_type(byte_t payload_type){

	switch (payload_type) {
		case GATEWAY_PAYLOAD_SIGNUP:
			return TRUE;
		case GATEWAY_PAYLOAD_REQUEST:
			return FALSE;
		case GATEWAY_PAYLOAD_CHALLENGE:
			return TRUE;
		case GATEWAY_PAYLOAD_RESPONSE:
			return FALSE;
		case GATEWAY_PAYLOAD_GRANTED:
			return TRUE;
		case GATEWAY_PAYLOAD_CONTROL:				// Note: Control not implemented
			return FALSE;
		case GATEWAY_PAYLOAD_SIGNUP_RESP:
			return FALSE;							// Signup responses are never sent FROM the gateway, only TO the gateway
		default:									// Unknown payload type
			return FALSE;
			break;
	}
}

bool_t gw_check_payload_len(uint16_t len, byte_t payload_type){

	switch (payload_type) {
		case GATEWAY_PAYLOAD_SIGNUP:
			return len == LEN_GATEWAY_PAYLOAD_SIGNUP;

		case GATEWAY_PAYLOAD_REQUEST:
			// Should NEVER get in here, because if the payload type is GATEWAY_PAYLOAD_REQUEST, then we should not have even called rx_check_len!
			while(TRUE){
				__NOP();
			}
			return len == LEN_GATEWAY_PAYLOAD_REQUEST;

		case GATEWAY_PAYLOAD_CHALLENGE:
			return len == LEN_GATEWAY_PAYLOAD_CHALLENGE;

		case GATEWAY_PAYLOAD_RESPONSE:
			// Should NEVER get in here, because if the payload type is GATEWAY_PAYLOAD_RESPONSE, then we should not have even called rx_check_len!
			while(TRUE){
				__NOP();
			}
			return len == LEN_GATEWAY_PAYLOAD_RESPONSE;

		case GATEWAY_PAYLOAD_GRANTED:				// Note: Control not implemented
			return len == LEN_GATEWAY_PAYLOAD_GRANTED;

		case GATEWAY_PAYLOAD_CONTROL:
			// Should NEVER get in here, because GATEWAY_PAYLOAD_CONTROL is not implemented
			while(TRUE){
				__NOP();
			}
			return len == LEN_GATEWAY_PAYLOAD_CONTROL;

		case GATEWAY_PAYLOAD_SIGNUP_RESP:
			// Should NEVER get in here, because if the payload type is GATEWAY_PAYLOAD_SIGNUP_RESP, then we should not have even called rx_check_len!
			while(TRUE){
				__NOP();
			}
			return len == LEN_GATEWAY_PAYLOAD_SIGNUP_RESP;


		default:										// Unknown payload type
			return FALSE;

	}

}

void gw_task_receive(msg_t* msg_state){
	static state_t state = STATE_GET_PAYLOAD_TYPE;

	bool_t valid;														// Set to 1 if value is in permissible range

	if(state == STATE_GET_PAYLOAD_TYPE){

		byte_t b;
		if(rb_consume_one(&us2_rx_rbuf, &b)){
			valid = gw_check_payload_type(b);
			if(valid){
				msg_state->payload_type = b;
				state = STATE_GET_PAYLOAD_LEN;
			}
			else{																				// Case: Bad/unknown payload type
				// TODO: Somehow notify for resynchronization and debug purposes!
				__NOP();
			}
		}

	}

	if(state == STATE_GET_PAYLOAD_LEN){
		uint16_t len;

		if(rb_consume(&us2_rx_rbuf, &len, 2)){														// Try to consume 2 bytes from receive buffer
			valid = gw_check_payload_len(len, msg_state->payload_type);
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
		if(rb_consume(&us2_rx_rbuf, msg_state->p_payload, len)){
			state = STATE_GET_PAYLOAD_TYPE;
			gw_flags.rx_done = TRUE;
			test_cnt++;
		}
	}
}
