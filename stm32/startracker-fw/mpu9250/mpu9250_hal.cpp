/*
 * mpu9250_hal.cpp
 *
 *  Created on: Nov 14, 2023
 *      Author: ftobler
 *
 * Ported from https://github.com/kriswiner/MPU9250
 */


#include "mpu9250_hal.h"
#include "scheduler.h"
#include "stm32_hal.h"



extern I2C_HandleTypeDef hi2c3;
#define hi2c hi2c3


void wait(float seconds) {
	uint32_t milliseconds = seconds * 1000;
	scheduler_task_sleep(milliseconds);
}


void delay(int milliseconds) {
	scheduler_task_sleep(milliseconds);
}

I2C::I2C() {

}

void I2C::write(uint8_t adr, const void* data, uint16_t len, uint8_t end) {
	HAL_I2C_Master_Transmit(&hi2c, adr, (uint8_t*)data, len, 100);
}

void I2C::read(uint8_t adr, void* data, uint16_t len, uint8_t end) {
	HAL_I2C_Master_Receive(&hi2c, adr, (uint8_t*)data, len, 100);
}
