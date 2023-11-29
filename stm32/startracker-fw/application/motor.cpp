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
#include "scheduler.h"


#define set_command_flag(flag)   motor.flags = (Motor_command_flags_t)(motor.flags |  (flag))
#define reset_command_flag(flag) motor.flags = (Motor_command_flags_t)(motor.flags & ~(flag))

#define N 4  //number of coefficients. Order is N-1
#define TICK_TIME (0.00025f)

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim5;
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

static void enable_disable_motors(uint32_t enable);
static void assign_trajectory();
static void run_home();
static void run_motors();


void motor_init() {
	//set all the Direction pins
	steppers[0].dir_port = TMC1_DIR_GPIO_Port;
	steppers[0].dir_pin  = TMC1_DIR_Pin;
	steppers[1].dir_port = TMC2_DIR_GPIO_Port;
	steppers[1].dir_pin  = TMC2_DIR_Pin;
	steppers[2].dir_port = TMC3_DIR_GPIO_Port;
	steppers[2].dir_pin  = TMC3_DIR_Pin;

	//set all the step pins
	steppers[0].step_port = TMC1_STEP_GPIO_Port;
	steppers[0].step_pin  = TMC1_STEP_Pin;
	steppers[1].step_port = TMC2_STEP_GPIO_Port;
	steppers[1].step_pin  = TMC2_STEP_Pin;
	steppers[2].step_port = TMC3_STEP_GPIO_Port;
	steppers[2].step_pin  = TMC3_STEP_Pin;

	//assign the uarts to all motors
	steppers[0].uart = &huart2;
	steppers[1].uart = &huart2;
	steppers[2].uart = &huart2;

	//cleanly init the power state
	enable_disable_motors(0);
}
void motor_update() {


	switch (motor.state) {
	case MOTOR_MODE_NONE:
		if (motor.flags & MOTOR_DO_HOME) {
			reset_command_flag(MOTOR_DO_HOME);
			motor.state = MOTOR_MODE_HOMING;
			home_timer = 0;
		}
		break;
	case MOTOR_MODE_HOMING:
		run_home();
		home_timer++;
		if (home_timer > 40000) {
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
	}

	enable_disable_motors(motor.state != MOTOR_MODE_NONE);

}



void motor_set_trajectory(Trajectory* trajectory) {
	new_trajectory = *trajectory;
	set_command_flag(MOTOR_DO_ASSIGN_TRAJECTORY);
}

void motor_do(Motor_command_flags_t cmd) {
	set_command_flag(cmd);
}


static void enable_disable_motors(uint32_t enable) {
	//this implements a simple state machine for ON and OFF of motors
	//the state machine is not in a traditional form.
	//it implements extended Enable and Disable functions for the motors
	static uint32_t old = -1; //force state transition on first update
	enable = enable ? 1 : 0; //clean up input variable

	if (old != enable) {
		old = enable;

		if (enable) {

			// MOTOR ENABLE SEQUENCE

			//disable motors
			HAL_GPIO_WritePin(TMC_ENABLE_GPIO_Port, TMC_ENABLE_Pin, GPIO_PIN_SET);

			//enable 12V power boosting. The 4.8V may not be enough for the drivers to start properly.
			//there is an issue that enabling the 12V booster can stress the 5.2V power booster and the battery
			//management so much that it shuts down because of overcurrent.
			//  =>  Simulations have shown that the 12V booster needs about ~100us to load all secondary capacitors,
			//      which is a considerable amount of charge. The solution is to enable it multiple times for 5us
			//      at one time with 1ms pauses in between.
			scheduler_task_sleep(1);
			for (int i = 0; i < 200; i++) {
				HAL_GPIO_WritePin(MOTOR_BOOST_ENABLE_GPIO_Port, MOTOR_BOOST_ENABLE_Pin, GPIO_PIN_SET);
				uint32_t micros = htim5.Instance->CNT + 5;
				while (htim5.Instance->CNT < micros);
				HAL_GPIO_WritePin(MOTOR_BOOST_ENABLE_GPIO_Port, MOTOR_BOOST_ENABLE_Pin, GPIO_PIN_RESET);
				scheduler_task_sleep(1);
			}
			HAL_GPIO_WritePin(MOTOR_BOOST_ENABLE_GPIO_Port, MOTOR_BOOST_ENABLE_Pin, GPIO_PIN_SET);

			//wait for it to get stable and TMC to boot up
			scheduler_task_sleep(200);

			//send TMC init sequence. Since the data line is shared this needs only be done on one motor.
			steppers[0].send_init();

			//enable motors
			HAL_GPIO_WritePin(TMC_ENABLE_GPIO_Port, TMC_ENABLE_Pin, GPIO_PIN_RESET);

		} else {

			// MOTOR DISABLE SEQUENCE

			//disable motors
			HAL_GPIO_WritePin(TMC_ENABLE_GPIO_Port, TMC_ENABLE_Pin, GPIO_PIN_SET);

			//disable 12V power boosting
			HAL_GPIO_WritePin(MOTOR_BOOST_ENABLE_GPIO_Port, MOTOR_BOOST_ENABLE_Pin, GPIO_PIN_RESET);

		}
	}
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
//		steppers[i].set_to(-1);
		if (home_timer > 40000 / 2) {
			steppers[i].set_to(4);
		} else {
			steppers[i].set_to(-4);
		}
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

