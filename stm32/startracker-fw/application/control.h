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

typedef enum {
	CONTROL_IDLE,
	CONTROL_MEASURING,
	CONTROL_RUNNING
} Control_state_en;

typedef struct {
	float battery_voltage;
	Control_state_en state;
} Control_t;


void control_init();
void control_update();


#ifdef __cplusplus
}
#endif


#endif /* CONTROL_H_ */
