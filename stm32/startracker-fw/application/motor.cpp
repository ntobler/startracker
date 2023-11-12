/*
 * motor.cpp
 *
 *  Created on: Nov 12, 2023
 *      Author: ftobler
 */

#include "motor.h"
#include "tmc220x.h"
#include "stm32_hal.h"

#define N 4  //number of coefficients. Order is N-1

extern UART_HandleTypeDef huart2;
static TMC220X motor[3];
//polynom like in python. Example data.
//the order is c3, c2, c1, c0
static float polynom[3][N] = {{ 7.11859163e-12f,  1.78833188e-07f,  3.31593291e-03f, -1.45327218e-09f },
						      { 3.55347520e-11f,  5.26166407e-07f,  9.16399197e-03f, -6.48713382e-09f },
						      { 1.27991415e-11f,  1.70074972e-07f,  6.23996244e-03f, -1.54755701e-09f }};
//stepper motor target position in mm
static float position[3];

static uint32_t tick = 0;

void motor_init() {
	motor[0].uart = &huart2;
	motor[0].send_init();
}
void motor_update() {
	tick++;
	float time = tick * 0.00025f;

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
	}
}
