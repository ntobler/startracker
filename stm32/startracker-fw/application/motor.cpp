/*
 * motor.cpp
 *
 *  Created on: Nov 12, 2023
 *      Author: ftobler
 */

#include "motor.h"
#include "tmc220x.h"
#include "string.h"
#include "stm32_hal.h"
#include "main.h"


#define set_command_flag(flag)   motor.flags = (Motor_command_flags_t)(motor.flags |  (flag))
#define reset_command_flag(flag) motor.flags = (Motor_command_flags_t)(motor.flags & ~(flag))

#define N 4  //number of coefficients. Order is N-1
#define TICK_TIME (0.00025f)

extern UART_HandleTypeDef huart2;
static TMC220X steppers[3];
//polynom like in python. Example data.
//the order is c3, c2, c1, c0
//static float polynom[3][N] = {{ 7.11859163e-12f,  1.78833188e-07f,  3.31593291e-03f, -1.45327218e-09f },
//						      { 3.55347520e-11f,  5.26166407e-07f,  9.16399197e-03f, -6.48713382e-09f },
//						      { 1.27991415e-11f,  1.70074972e-07f,  6.23996244e-03f, -1.54755701e-09f }};

//stepper motor target position in mm
static float position[3];

static int32_t start_tick = 0;
static int32_t end_tick = 0;
static int32_t home_timer = 0;
static Trajectory new_trajectory;
static float polynom[3][N];
static float start_time = 0.0f;
static float end_time = 0.0f;
static int32_t tick = 0;
Motor_t motor = {MOTOR_MODE_NONE};


static int32_t get_start_tick();
static int32_t get_end_tick();

static void enable_motors();
static void disable_motors();
static void assign_trajectory();
static void run_home();
static void run_motors();


void motor_init() {
	steppers[0].uart = &huart2;
	steppers[0].send_init();
	disable_motors();
}
void motor_update() {


	switch (motor.state) {
	case MOTOR_MODE_NONE:
		if (motor.flags & MOTOR_DO_HOME) {
			reset_command_flag(MOTOR_DO_HOME);
			motor.state = MOTOR_MODE_HOMING;
			enable_motors();
			home_timer = 0;
		}
		break;
	case MOTOR_MODE_HOMING:
		run_home();
		if (home_timer > 2000) {
			motor.state = MOTOR_MODE_READY;
		}
		break;
	case MOTOR_MODE_READY:
		if (motor.flags & MOTOR_DO_HOME) {
			reset_command_flag(MOTOR_DO_HOME);
			motor.state = MOTOR_MODE_HOMING;
			home_timer = 0;
		} else
		if (motor.flags & MOTOR_DO_START && motor.trajectory_ready) {
			reset_command_flag(MOTOR_DO_START);
			motor.state = MOTOR_MODE_RUNNING;
		} else
		if (motor.flags & MOTOR_DO_DISABLE) {
			reset_command_flag(MOTOR_DO_DISABLE);
			motor.state = MOTOR_MODE_NONE;
			disable_motors();
		} else
		if (motor.flags & MOTOR_DO_ASSIGN_TRAJECTORY) {
			reset_command_flag(MOTOR_DO_ASSIGN_TRAJECTORY);
			assign_trajectory();
		}
		break;
	case MOTOR_MODE_RUNNING:
		if (motor.flags & MOTOR_DO_STOP) {
			reset_command_flag(MOTOR_DO_STOP);
			motor.state = MOTOR_MODE_READY;
		} else
		if (motor.flags & MOTOR_DO_PAUSE) {
			reset_command_flag(MOTOR_DO_PAUSE);
			motor.state = MOTOR_MODE_PAUSE;
		}
		run_motors();
		break;
	case MOTOR_MODE_PAUSE:
		if (motor.flags & MOTOR_DO_RESUME) {
			reset_command_flag(MOTOR_DO_RESUME);
			motor.state = MOTOR_MODE_RUNNING;
		} else
		if (motor.flags & MOTOR_DO_STOP) {
			reset_command_flag(MOTOR_DO_STOP);
			motor.state = MOTOR_MODE_READY;
		}
		break;
	default:
		motor.state = MOTOR_MODE_NONE;
		disable_motors();
	}

}



void motor_set_trajectory(Trajectory* trajectory) {
	new_trajectory = *trajectory;
	set_command_flag(MOTOR_DO_ASSIGN_TRAJECTORY);
}

void motor_control(Motor_command_flags_t cmd) {
	set_command_flag(cmd);
}

static void enable_motors() {
	HAL_GPIO_WritePin(TMC_ENABLE_GPIO_Port, TMC_ENABLE_Pin, GPIO_PIN_SET);
}

static void disable_motors() {
	HAL_GPIO_WritePin(TMC_ENABLE_GPIO_Port, TMC_ENABLE_Pin, GPIO_PIN_RESET);
}
static void assign_trajectory() {
	start_time = new_trajectory.start;
	end_time = new_trajectory.stop;
	memccpy(polynom, new_trajectory.coeffs, 3*N, sizeof(float));
	start_tick = get_start_tick();
	end_tick = get_end_tick();
	tick = start_tick;
}


static void run_home() {
	for (int i = 0; i < 3; i++) {
		steppers[i].set_to(-1);
		steppers[i].step_to(0);
	}
}

static void run_motors() {
	tick++;
	float time = tick * TICK_TIME;

	//calculate all squares and cubes and higher here. this goes from t^0 up to t^N
	float potents[N];
	potents[0] = 1.0f;
	for (int i = 1; i < N; i++) {
		potents[i] = time * potents[i-1];
	}

	//calculate the polynoms
	for (int i = 0; i < 3; i++) {
		float pos = 0.0f;
		for (int n = 0; n < N; n++) {
			pos += potents[n] * polynom[i][N - n - 1]; //need to invert the polynomial order
		}
		position[i] = pos;
		steppers[i].step_to(pos);
	}
}


static int32_t get_start_tick() {
	return start_time / TICK_TIME;
}

static int32_t get_end_tick() {
	return end_time / TICK_TIME;
}

