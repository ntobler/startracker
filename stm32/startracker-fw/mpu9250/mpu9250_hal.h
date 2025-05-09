/*
 * mpu9250_hal.h
 *
 *  Created on: Nov 14, 2023
 *      Author: ftobler
 */

#ifndef MPU9250_HAL_H_
#define MPU9250_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

void wait(float seconds);
void delay(int milliseconds);

class I2C {
private:
public:
	I2C();
	void write(uint8_t adr, const void* data, uint16_t len, uint8_t end);
	void read(uint8_t adr, void* data, uint16_t len, uint8_t end);
};





#ifdef __cplusplus
}
#endif

#endif /* MPU9250_HAL_H_ */
