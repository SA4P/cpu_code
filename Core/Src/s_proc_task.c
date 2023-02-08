/*
 * proc_task.c
 *
 *  Created on: Apr 3, 2022
 *      Author: SA4P Authors
 */

#include <s_proc_task.h>

uint16_t s_proc_get_access_type(byte_t* payload);
bool_t	 s_proc_set_req_state(uint16_t access_type, req_t *req_state);
void 	 s_proc_signup_set_dev_id(msg_t* msg_state);


bool_t s_task_process_msg(msg_t* msg_state, req_t* req_state, byte_t* key_buf, send_msg_t* o_msg){
	s_flags.rx_done = FALSE;

	if(msg_state->payload_type == SERVER_PAYLOAD_SIGNUP_RESP){
		// Case 1: Received a signup response from server


		// (1.1) Do NOT check if we are already signed up
//		// TODO: Handle case of duplicated signup more gracefully
//		if(dev_state.signed_up){
//			while(TRUE){
//				__NOP();
//			}
//		}


		// (1.2) Set device ID based on server's response
		s_proc_signup_set_dev_id(msg_state);

		// (1.3) Create GATEWAY signup response which is then to be forwarded to the gateway
		if(!gw_msg_build(GATEWAY_PAYLOAD_SIGNUP_RESP, req_state, msg_state->p_payload + DEVICE_ID_LEN, o_msg)){
			while(TRUE){
				__NOP();
			}
		}

		// (1.4) Set send flag, such that the gateway send task then later starts sending
		gw_flags.tx_rdy = TRUE;

		return TRUE;
	}

	if(msg_state->payload_type == SERVER_PAYLOAD_AUTH_RESP){
		// Case 2: Received authentication request from server

		// (2.1) Check if current request doesn't have expected state (we should be in the state "not authenticated").
		// TODO: Handle case where unexpected authentication response comes more gracefully
		if(req_state->status != STATUS_NOT_AUTHED){
			while(TRUE){
				__NOP();
			}
		}

		#ifdef PROFILING_MACRO_AUTH_TIME
			// 4: Received authentication response from server
			PROFILING_SET_COUNTER_E(4);
		#endif

		// (2.2) Update the request_state's status
		req_state->status = STATUS_AUTHED;

		// (2.3) Set response in request state
		memcpy(req_state->response, msg_state->p_payload, LEN_SERVER_PAYLOAD_AUTH_RESP);

		// (2.3) Build response message to be sent to the gateway. If an error occurs, we fail in an infinite loop
		if(!gw_msg_build(GATEWAY_PAYLOAD_RESPONSE, req_state, NULL, o_msg)){
			while(TRUE){
				__NOP();
			}
		}

		// (2.4) Set send flag, such that the gateway send task then later starts sending
		gw_flags.tx_rdy = TRUE;

		#ifdef PROFILING_MACRO_AUTH_TIME
			// 5: Created GATEWAY_PAYLOAD_RESPONSE from server's SERVER_PAYLOAD_AUTH_RESP ==> Ready to send to CPU
			PROFILING_SET_COUNTER_E(5);
		#endif

		return TRUE;
	}

	if(msg_state->payload_type == SERVER_PAYLOAD_CONTROL){
		// Not yet implemented
		while(TRUE){
			__NOP();
		}
	}

	return FALSE;
}

uint16_t s_proc_get_access_type(byte_t* payload){
	return ((uint16_t*)payload)[0];
}


void s_proc_signup_set_dev_id(msg_t* msg_state){
	memcpy(&dev_state.dev_id, msg_state->p_payload, DEVICE_ID_LEN);
	dev_state.signed_up = TRUE;

}


// - - - - - - - - - - - - - - - - -
//      LIKELY UNUSED FUNCTIONS
// - - - - - - - - - - - - - - - - -

//bool_t proc_set_req_state(uint16_t access_type, req_t *req_state){
//
//	if (access_type < 0 || access_type > CONTROL_ACTUATOR_1){															// Received malformed access type ==> Ignore request
//		__NOP();																											// NOP to break on for debugging purposes
//		return FALSE;
//	}
//
//	req_state->access_type = access_type;
//
//
//	// Sample random challenge of length CHALLENGE_LEN (currently 16 bytes)
//	rng_get_randomness(req_state->challenge, CHALLENGE_LEN);
//
//	// Set receive time
//	req_state->timer_set_time = HAL_GetTick();
//
//	// Set new request state to true
//	req_state->valid = TRUE;
//
//	return TRUE;
//}
//
//err_t proc_check_response(msg_t* msg_state, req_t* req_state, byte_t* key_buf){
//	if(!req_state->valid){
//		return -1;
//	}
//
//	if(HAL_GetTick() - req_state->timer_set_time > req_state->timeout){
//		req_state->valid = FALSE;
//		return -2;
//	}
//
//	byte_t hmac_input[ACCESS_TYPE_LEN + CHALLENGE_LEN];
//	memcpy(hmac_input, &req_state->access_type, ACCESS_TYPE_LEN);
//	memcpy(hmac_input + 2, req_state->challenge, CHALLENGE_LEN);
//
//	byte_t reference_mac[HMAC_OUTPUT_SIZE];
//	hash_hmac(key_buf, hmac_input, HMAC_KEY_LEN, ACCESS_TYPE_LEN + CHALLENGE_LEN, reference_mac);
//
//	byte_t* resp = msg_state->p_payload;
//
//	// Compare locally computed reference HMAC with that extracted from the payload. If the match, we return 0 (no error), else -3 (HMAC verifiaction error)
//	int difference = memcmp(reference_mac, resp, HMAC_OUTPUT_SIZE);
//	if(difference == 0){
//
//		return 0;
//	}
//	else{
//		return -3;
//	}
//}
