/*
 * perf.h
 *
 *  Created on: Nov 17, 2023
 *      Author: ftobler
 */

#ifndef PERF_H_
#define PERF_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32_hal.h"


typedef struct {
	uint32_t start;
	uint32_t sum;
} Perf_private;

class Perf {
private:
	Perf_private _private = {0};
public:
	uint32_t last = 0;
	uint32_t average = 0;
	uint32_t cycles = 0;
	Perf();
	void start();
	void end();
};


#ifdef __cplusplus
}
#endif

#endif /* PERF_H_ */
