/*
 * proc_task.h
 *
 *  Created on: Apr 3, 2022
 *      Author: SA4P Authors
 */

#ifndef INC_GW_PROC_TASK_H_
#define INC_GW_PROC_TASK_H_

#include "main.h"

bool_t gw_task_process_msg(msg_t* msg_state, req_t* req_state, byte_t* key_buf, send_msg_t* o_msg);

#endif /* INC_GW_PROC_TASK_H_ */
