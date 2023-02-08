/*
 * send_task.h
 *
 *  Created on: Apr 4, 2022
 *      Author: SA4P Authors
 */

#ifndef INC_SEND_TASK_H_
#define INC_SEND_TASK_H_

#include "main.h"

bool_t s_task_send(send_msg_t* o_msg);
bool_t gw_task_send(send_msg_t* o_msg);

//bool_t task_send(conns_t conn_selector, send_msg_t* o_msg);

#endif /* INC_SEND_TASK_H_ */
