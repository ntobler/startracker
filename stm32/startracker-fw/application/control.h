/*
 * control.h
 *
 *  Created on: Nov 17, 2023
 *      Author: ftobler
 */

#ifndef CONTROL_H_
#define CONTROL_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stdint.h"


typedef enum {
	CONTROL_IDLE,
	CONTROL_BOOTING,
	CONTROL_MEASURING,
	CONTROL_RUNNING
} Control_state_en;

typedef struct {
	float battery_voltage;
	uint8_t charger_charging;
	uint8_t charger_done;
	Control_state_en state;
	uint8_t rpi_running;
} Control_t;


void control_init();
void control_update();
void control_action_capture();
void control_action_track();
void control_action_reset();
void control_action_shutdown();


#ifdef __cplusplus
}
#endif


#endif /* CONTROL_H_ */
