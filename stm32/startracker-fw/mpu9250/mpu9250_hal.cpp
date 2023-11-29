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
#include "application.h"



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
	scheduler_event_clear(EVENT_TASK_I2C_COMPLETE);
	HAL_I2C_Master_Transmit_IT(&hi2c, adr, (uint8_t*)data, len);
	scheduler_event_wait_timeout(EVENT_TASK_I2C_COMPLETE, 3);
}

void I2C::read(uint8_t adr, void* data, uint16_t len, uint8_t end) {
	scheduler_event_clear(EVENT_TASK_I2C_COMPLETE);
	HAL_I2C_Master_Receive_IT(&hi2c, adr, (uint8_t*)data, len);
	scheduler_event_wait_timeout(EVENT_TASK_I2C_COMPLETE, 3);
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* _hi2c) {
	if (_hi2c->Instance == hi2c3.Instance) {
		scheduler_event_set(ID_TASK_UI, EVENT_TASK_I2C_COMPLETE);
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* _hi2c) {
	if (_hi2c->Instance == hi2c3.Instance) {
		scheduler_event_set(ID_TASK_UI, EVENT_TASK_I2C_COMPLETE);
	}
}

