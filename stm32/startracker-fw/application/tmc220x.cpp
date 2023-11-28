/*
 * tmc220x.cpp
 *
 *  Created on: Nov 11, 2023
 *      Author: ftobler
 */

#include "tmc220x.h"
#include "scheduler.h"


#define REG_GCONF (0x00)
#define REG_CHOPCONF (0x6C)
#define REG_VELOCITY_DEP_CNTRL (0x10)
#define REG_WRITE (0x80)
#define REG_READ  (0x00)

//https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2202_TMC2208_TMC2224_datasheet_rev1.13.pdf
//page 23
#define gconf_I_scale_analog   (0x01 << 0)
#define gconf_internal_Rsense  (0x01 << 1)
#define gconf_en_SpreadCycle   (0x01 << 2)
#define gconf_shaft_reverse    (0x01 << 3)
#define gconf_index_otpw       (0x01 << 4)
#define gconf_index_step       (0x01 << 5)
#define gconf_pdn_disable      (0x01 << 6)
#define gconf_mstep_reg_select (0x01 << 7)
#define gconf_multistep_filt   (0x01 << 8)
#define gconf_test_mode        (0x01 << 9)

typedef struct __attribute__((__packed__)) {
	uint8_t sync_reserved; //10
	uint8_t slave_addr;    //0
	uint8_t register_addr; //highest byte = 1 for write, GCONF = 0x00
	uint32_t data;
	uint8_t crc;
} TMC_datagram_t;


static uint32_t byteswap(uint32_t data);


TMC220X::TMC220X() {
	_pos = 0;
}

void TMC220X::send_init() {

	//send general config
	send_data(REG_GCONF,
//			gconf_I_scale_analog |
			gconf_internal_Rsense |
			gconf_en_SpreadCycle |
//			gconf_shaft_reverse |
//			gconf_index_otpw |
//			gconf_index_step |
			gconf_pdn_disable |
			gconf_mstep_reg_select |
			gconf_multistep_filt |
//			gconf_test_mode |
			0
			);

	//send chop config
	send_data(REG_CHOPCONF, 0x00000053);  //Reset default=0x10000053

	//send pwm config
	send_data(REG_CHOPCONF, 0xC10D0024);  //Reset default=0xC10D0024


//	send_data(REG_VELOCITY_DEP_CNTRL, 2 + (4 << 8) + (1 << 16));

}

void TMC220X::set_to(int32_t pos) {
	_pos = pos;
}
void TMC220X::step_to(int32_t pos) {
	if (_pos == pos) {
		return; //nothing to do
	}
	if (_pos > pos) {
		HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
		while (_pos > pos) {
			{volatile int i = 10; while (i) i--;}
			HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_SET);
			pos++;
			{volatile int i = 10; while (i) i--;}
			HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);
		}
	} else {
		HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
		while (_pos < pos) {
			{volatile int i = 10; while (i) i--;}
			HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_SET);
			pos--;
			{volatile int i = 10; while (i) i--;}
			HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);
		}
	}
}


void TMC220X::send_data(uint8_t reg, uint32_t data) {
	TMC_datagram_t d = {0};
	d.sync_reserved = 0b00000101;
	d.slave_addr = 0;
	d.register_addr = REG_WRITE | reg;
	d.data = byteswap(data);
	swuart_calcCRC((uint8_t*)&d, sizeof(d));
	HAL_UART_Transmit(uart, (uint8_t*)&d,  sizeof(d), 100);
	scheduler_task_sleep(50);
}


void TMC220X::swuart_calcCRC(uint8_t* datagram, uint32_t datagramLength) {
	 uint8_t* crc = datagram + (datagramLength-1); // CRC located in last byte of message
	 uint8_t currentByte;
	 *crc = 0;
	 for (uint32_t i = 0; i < (datagramLength-1); i++) { // Execute for all bytes of a message
		 currentByte = datagram[i]; // Retrieve a byte to be sent from Array
		 for (uint32_t j = 0; j < 8; j++) {
			 if ((*crc >> 7) ^ (currentByte&0x01)) { // update CRC based result of XOR operation
				 *crc = (*crc << 1) ^ 0x07;
			 } else {
				 *crc = (*crc << 1);
			 }
		 currentByte = currentByte >> 1;
		 } // for CRC bit
	 } // for message byte
}


static uint32_t byteswap(uint32_t data) {
	//swap high and low bytes
	return ((data >> 24) & 0xFF) | ((data >> 8) & 0xFF00) | ((data << 8) & 0xFF0000) | ((data << 24) & 0xFF000000);
}

