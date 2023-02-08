/*
 * button_task.c
 *
 *  Created on: Jun 13, 2022
 *      Author: SA4P Authors
 */

#include "button_task.h"

static bool_t state_button_pressed = FALSE;

static bool_t state_signup_requested = FALSE;

static int bounce_counter = 0;

static bool_t profiling_button_pressed = FALSE;

bool_t button_task_check_pressed(req_t* req_state, send_msg_t* o_msg){

	#ifdef PROFILING_BUTTON_PRESS_100MS
		if(profiling_button_pressed){
			HAL_Delay(1000);
			goto LABEL_PROFILING_BUTTON_PRESS;
		}
	#endif

	// (1) Check button register
	bool_t current_button_value = READ_BIT(gpio_button_port->IDR, gpio_button_pin) != FALSE;
	if(current_button_value == FALSE){
		// If button currently not considered pressed, reset pressed state such that when it is pressed the next time we will actually register it
		state_button_pressed = FALSE;
		return FALSE;
	}

	// If we reach here, the button register currently indicates that the button is pressed

	// (2) Check if the button state was already considered pressed previously ==> Don't want to double-register requests, so just do nothing then
	if(state_button_pressed == TRUE){
		return FALSE;
	}

	// If we reach here, then the button is currently considered pressed but was not considered so in the previous iteration of the function

	LABEL_PROFILING_BUTTON_PRESS:

	// (3) Set button state as pressed
	state_button_pressed = TRUE;

	#ifdef PROFILING_MACRO_AUTH_TIME
		// 0: New Button Press
		PROFILING_SET_COUNTER_E(0);
	#endif
	bounce_counter ++;

	// (4) Set request state
	req_state->access_type = 0;		// Currently, only always use access type 0
	req_state->status      = STATUS_NOT_CHALLENGED;

	// (5) Generate request message for the gateway
	if(!gw_msg_build(GATEWAY_PAYLOAD_REQUEST, req_state, NULL, o_msg)){
		// Case: Message building failed ==> Infinite error loop
		while(TRUE){
			__NOP();
		}
	}

	// (6) Set transmit ready flag, such that gateway send task can start sending
	gw_flags.tx_rdy = TRUE;

	profiling_button_pressed = TRUE;

	#ifdef PROFILING_MACRO_AUTH_TIME
		// 1: Request generated
		PROFILING_SET_COUNTER_E(1);
	#endif
	return TRUE;
}
