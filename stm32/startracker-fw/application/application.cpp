/*
 * application.cpp
 *
 *  Created on: Nov 7, 2023
 *      Author: cchtofl01
 */

#include "main.h"
#include "application.h"
#include "scheduler.h"
#include "ssd1306.h"
#include "string"
#include "serial.h"
#include "serial_vcp.h"
#include "usbd_cdc_if.h"
#include "ui.h"


static uint32_t vpc_send_cb(uint8_t* buf, uint32_t len);

static uint8_t stack0[4096];
static uint8_t stack1[4096];
static uint8_t stack2[4096];
static void task0();
static void task1();
static void task2();

extern uint32_t os_started;
extern UART_HandleTypeDef huart1;

Serial serial_rpi;
SerialVCP serial_usb(vpc_send_cb);



void app_init() {
	serial_rpi.init(&huart1);
	__enable_irq();

	scheduler_addTask(0,  task0, stack0,  4096);
	scheduler_addTask(1,  task1, stack1,  4096);
	scheduler_addTask(2,  task2, stack2,  4096);
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

static void task0() {
	while (1) {
		//IDLE TASK
//		if (serial_usb.available()) {
//			char c = serial_usb.read();
//			serial_usb.write(c);
//		}
		int avail = serial_usb.available();
		if (avail > 256) {
			avail = 256;
		}
		if (avail) {
			static uint8_t buf[256];
			serial_usb.readBuf(buf, avail);
			serial_usb.writeBuf(buf, avail);
			//scheduler_task_sleep(10);
		}
	}
}

static void task1() {
	ui_init();
	while (1) {
		ui_update();
		scheduler_task_sleep(50);
	}
}


static void task2() {
	while (1) {
		scheduler_task_sleep(3000);
	}
}


