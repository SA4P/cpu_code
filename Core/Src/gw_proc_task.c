/*
 * proc_task.c
 *
 *  Created on: Apr 3, 2022
 *      Author: SA4P Authors
 */

#include <gw_proc_task.h>

uint16_t gw_proc_get_access_type(byte_t* payload);
bool_t	 gw_proc_set_req_state(uint16_t access_type, req_t *req_state);
void 	 gw_proc_signup_msg(msg_t* msg_state);
uint16_t gw_proc_granted_msg(msg_t* msg_state);

bool_t gw_task_process_msg(msg_t* msg_state, req_t* req_state, byte_t* key_buf, send_msg_t* o_msg){
	gw_flags.rx_done = FALSE;

	if(msg_state->payload_type == GATEWAY_PAYLOAD_SIGNUP){
		// Case 1: Received a signup message from gateway

		paired = FALSE;
		// (1.1) Do NOT check if we are already signed up! We should allows signup requests by the gateway as a means of re-pairing!
//		// TODO: Handle case of duplicated signup more gracefully
//		if(dev_state.signed_up){
//			while(TRUE){
//				__NOP();
//			}
//		}

		// (1.2) Process signup message
		gw_proc_signup_msg(msg_state);

		// (1.3) Create signup request to be sent to server
		if(!s_msg_build(SERVER_PAYLOAD_SIGNUP_REQ, req_state, msg_state, o_msg)){
			while(TRUE){
				__NOP();
			}
		}

		// (1.4) Set flag signaling that signup request can be sent to server
		s_flags.tx_rdy = TRUE;
		return TRUE;

	}

	if(msg_state->payload_type == GATEWAY_PAYLOAD_CHALLENGE){
		// Case 2: Received authentication request from server

		// (2.1) Check if current request doesn't have expected state (we should be in the state "not challenged").
		// TODO: Handle case where unexpected challenge comes more gracefully
		if(req_state->status != STATUS_NOT_CHALLENGED && paired){
			while(TRUE){
				__NOP();
			}
		}

	#ifdef PROFILING_MACRO_AUTH_TIME
		// 2: Challenge received
		PROFILING_SET_COUNTER_E(2);
	#endif

		paired = TRUE;


		// (2.2) Update the request_state's status
		req_state->status = STATUS_NOT_AUTHED;

		// (2.3) Set challenge in request state
		memcpy(req_state->challenge, msg_state->p_payload, LEN_GATEWAY_PAYLOAD_CHALLENGE);

		// (2.3) Build auth_req (authentication request) message to be sent to the server. If an error occurs, we fail in an infinite loop.
		if(!s_msg_build(SERVER_PAYLOAD_AUTH_REQ, req_state, NULL, o_msg)){
			while(TRUE){
				__NOP();
			}
		}

		// (2.4) Set send flag, such that the gateway send task then later starts sending
		s_flags.tx_rdy = TRUE;

		#ifdef PROFILING_MACRO_AUTH_TIME
			// 3: Challenge encapsulated in SERVER_PAYLOAD_AUTH_REQ ==> Ready to send to server
			PROFILING_SET_COUNTER_E(3);
		#endif

		return TRUE;
	}

	if(msg_state->payload_type == GATEWAY_PAYLOAD_GRANTED){
		// Case 3: Received granted message, i.e. can access the ressource

		#ifdef PROFILING_MACRO_AUTH_TIME
			// 6: Received grant from GW
			PROFILING_SET_COUNTER_E(6);
		#endif

		uint16_t access_type = gw_proc_granted_msg(msg_state);
//		access_type += 0;
		// TODO: Access ressource

		return TRUE;

	}

	if(msg_state->payload_type == GATEWAY_PAYLOAD_CONTROL){
		// Not yet implemented
		while(TRUE){
			__NOP();
		}
	}

	return FALSE;
}

uint16_t gw_proc_get_access_type(byte_t* payload){
	return ((uint16_t*)payload)[0];
}


void gw_proc_signup_msg(msg_t* msg_state){
	memcpy(&dev_state.dev_type, msg_state->p_payload, 2);
}

uint16_t gw_proc_granted_msg(msg_t* msg_state){
	uint16_t granted_access_type;
	memcpy(&granted_access_type, msg_state->p_payload, LEN_GATEWAY_PAYLOAD_GRANTED);
	return granted_access_type;
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
