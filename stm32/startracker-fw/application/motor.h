/*
 * motor.h
 *
 *  Created on: Nov 12, 2023
 *      Author: ftobler
 */

#ifndef MOTOR_H_
#define MOTOR_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "out.h"

typedef enum {
	MOTOR_DO_STOP = 0x01,
	MOTOR_DO_PAUSE = 0x02,
	MOTOR_DO_HOME = 0x04,
	MOTOR_DO_RESUME = 0x08,
	MOTOR_DO_START = 0x10,
	MOTOR_DO_DISABLE = 0x20,
	MOTOR_DO_ASSIGN_TRAJECTORY = 0x40,
} Motor_command_flags_t;

typedef enum {
	MOTOR_MODE_NONE,
	MOTOR_MODE_HOMING,
	MOTOR_MODE_READY,
	MOTOR_MODE_RUNNING,
	MOTOR_MODE_PAUSE,
} Motor_state_en;

typedef struct {
	Motor_state_en state;
	Motor_command_flags_t flags;
	uint32_t trajectory_ready;
} Motor_t;


void motor_init();
void motor_update();
void motor_set_trajectory(Trajectory* trajectory);
void motor_control(Motor_command_flags_t cmd);


#ifdef __cplusplus
}
#endif


#endif /* MOTOR_H_ */
