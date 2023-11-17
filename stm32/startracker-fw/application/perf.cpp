/*
 * perf.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: ftobler
 */

#include "perf.h"



extern TIM_HandleTypeDef htim5;

#define get_micros() (htim5.Instance->CNT)

Perf::Perf() {

}


void Perf::start() {
	_private.start = get_micros();
}


void Perf::end() {
	uint32_t end = get_micros();
	uint32_t t = end - _private.start;
	cycles++;
	uint32_t sum = _private.sum + t;
	_private.sum = sum;
	last = t;
	average = sum / cycles;
}
