/*
 * application.h
 *
 *  Created on: Nov 7, 2023
 *      Author: cchtofl01
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"


enum {
	ID_TASK_IDLE = 0,

	ID_TASK_UI = 1,
	EVENT_TASK_UPDATE = 0x01,

	ID_TASK_CONTROL = 2,

	ID_TASK_MOTOR = 3,
	EVENT_TASK_MOTOR_TIMER = 0x01,
};

void app_init();
void uart_isr();
void vcp_receive_cb(uint8_t* buf, uint32_t len);
void vcp_send_cmplt_cb();

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_H_ */
