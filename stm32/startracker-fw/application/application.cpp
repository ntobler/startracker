/*
 * application.cpp
 *
 *  Created on: Nov 7, 2023
 *      Author: cchtofl01
 */

#include <mpu9250.h>
#include "main.h"
#include "application.h"
#include "scheduler.h"
#include "ssd1306.h"
#include "string"
#include "serial.h"
#include "serial_vcp.h"
#include "usbd_cdc_if.h"
#include "ui.h"
#include "motor.h"
#include "imu.h"
#include "perf.h"
#include "control.h"


typedef struct {
	Perf ui;
	Perf imu;
	Perf motor;
	Perf control;
} Perf_monitor;


static uint32_t vpc_send_cb(uint8_t* buf, uint32_t len);

#define STACK_SIZE (4096)

static uint8_t stack_idle[STACK_SIZE];
static uint8_t stack_ui[STACK_SIZE];
static uint8_t stack_control[STACK_SIZE];
static uint8_t stack_motor[STACK_SIZE];
static void task_idle();
static void task_ui();
static void task_control();
static void task_motor();

extern uint32_t os_started;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim4;   //fast timer for motor control update
extern TIM_HandleTypeDef htim5;   //microseconds counter up to 4294967295. Used by imu and perf
extern I2C_HandleTypeDef hi2c3;


Serial serial_rpi;
SerialVCP serial_usb(vpc_send_cb);
Perf_monitor perf;



void app_init() {
	HAL_TIM_Base_Start(&htim5);
	serial_rpi.init(&huart1);
	__enable_irq();

	scheduler_addTask(ID_TASK_IDLE,    task_idle,    stack_idle,    STACK_SIZE);
	scheduler_addTask(ID_TASK_UI,      task_ui,      stack_ui,      STACK_SIZE);
	scheduler_addTask(ID_TASK_CONTROL, task_control, stack_control, STACK_SIZE);
	scheduler_addTask(ID_TASK_MOTOR,   task_motor,   stack_motor,   STACK_SIZE);

	os_started = 1;
	scheduler_join();

}

void uart_isr() {
	serial_rpi.ISR(&huart1);
}

void vcp_receive_cb(uint8_t* buf, uint32_t len) {
	serial_usb.vcp_receive_cb(buf, len);
}

void vcp_send_cmplt_cb() {
	serial_usb.vcp_send_cmplt_cb();
}

static uint32_t vpc_send_cb(uint8_t* buf, uint32_t len) {
	uint8_t res = CDC_Transmit_FS(buf, len);
	return res != USBD_OK;
}

static void task_idle() {
	while (1) {
		//IDLE TASK

		int avail = serial_usb.available();
		if (avail) {
			if (avail > 256) {
				avail = 256;
			}
			static uint8_t buf[256];
			serial_usb.readBuf(buf, avail);
			serial_usb.writeBuf(buf, avail);
			//scheduler_task_sleep(10);
		}
//		__WFI();
	}
}

static void task_ui() {
	scheduler_task_sleep(200);
	ui_init();
	imu_init();
	while (1) {
		//Wait for UI update event. This should arrive cyclic.
		//Waiting is done with a timeout so the task is used to poll the IMU sensor
		//which is on the same I2C bus as the display.
		uint32_t event = scheduler_event_wait_timeout(EVENT_TASK_UPDATE, 1);
		if (event & EVENT_TASK_UPDATE) {
			perf.ui.start();
			ui_update();
			perf.ui.end();
		}

		//only update IMU when I2C is ready. tha means the DMA transfer(s) from the display are finished.
		if (hi2c3.State == HAL_I2C_STATE_READY) {
			perf.imu.start();
			imu_update();
			perf.imu.end();
		}
	}
}


static void task_control() {
	HAL_GPIO_WritePin(POWER_ENABLE_GPIO_Port, POWER_ENABLE_Pin, GPIO_PIN_SET);
	control_init();
	while (1) {
		perf.control.start();
		control_update();
		scheduler_event_set(ID_TASK_UI, EVENT_TASK_UPDATE);
		perf.control.end();
		scheduler_task_sleep(100);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
}

static void task_motor() {
	HAL_TIM_Base_Start_IT(&htim4);
	motor_init();
	while (1) {
//		scheduler_task_sleep(3000);
		uint32_t event = scheduler_event_wait(EVENT_TASK_MOTOR_TIMER);
		if (event & EVENT_TASK_MOTOR_TIMER) {
			perf.motor.start();
			motor_update();
			perf.motor.end();
		}
	}
}


