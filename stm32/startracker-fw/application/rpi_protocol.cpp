/*
 * rpi_protocol.cpp
 *
 *  Created on: Nov 12, 2023
 *      Author: ftobler
 */


#include "rpi_protocol.h"
#include "string.h"
#include "serial.h"
#include "scheduler.h"



enum {
	RPI_BUFFER_SIZE = 256,

	RPI_CMD_Reserved = 0,
	RPI_CMD_GetStatus = 1,
	RPI_CMD_SetSettings = 2,
	RPI_CMD_CalcTrajectory = 3,
	RPI_CMD_Shutdown = 4,
};


extern Serial serial_rpi;

static uint8_t rpi_buffer[RPI_BUFFER_SIZE];

static uint32_t protocol_send_receive(uint8_t cmd, const uint8_t* request_dat, uint32_t request_size, uint8_t* response_dat, uint32_t response_size);
static uint16_t calc_crc16(const uint8_t* data, uint32_t len);



Status* rpi_status() {
	const uint8_t cmd = RPI_CMD_GetStatus;
	const uint8_t* request_dat = 0;
	uint32_t request_size = 0;
	uint32_t response_size = sizeof(Status);
	uint32_t ret = protocol_send_receive(cmd, request_dat, request_size, rpi_buffer, response_size);
	if (ret == 0) {
		return (Status*)&rpi_buffer;
	}
	return 0;
}

Acknowledge* SetSettings(Settings* settings) {
	const uint8_t cmd = RPI_CMD_SetSettings;
	const uint8_t* request_dat = (uint8_t*)settings;
	uint32_t request_size = sizeof(Settings);
	uint32_t response_size = sizeof(Acknowledge);
	uint32_t ret = protocol_send_receive(cmd, request_dat, request_size, rpi_buffer, response_size);
	if (ret == 0) {
		return (Acknowledge*)&rpi_buffer;
	}
	return 0;
}

Trajectory * CalcTrajectory() {
	const uint8_t cmd = RPI_CMD_CalcTrajectory;
	const uint8_t* request_dat = 0;
	uint32_t request_size = 0;
	uint32_t response_size = sizeof(Trajectory);
	uint32_t ret = protocol_send_receive(cmd, request_dat, request_size, rpi_buffer, response_size);
	if (ret == 0) {
		return (Trajectory*)&rpi_buffer;
	}
	return 0;
}

Acknowledge* Shutdown() {
	const uint8_t cmd = RPI_CMD_Shutdown;
	const uint8_t* request_dat = 0;
	uint32_t request_size = 0;
	uint32_t response_size = sizeof(Acknowledge);
	uint32_t ret = protocol_send_receive(cmd, request_dat, request_size, rpi_buffer, response_size);
	if (ret == 0) {
		return (Acknowledge*)&rpi_buffer;
	}
	return 0;
}


static uint32_t protocol_send_receive(uint8_t cmd, const uint8_t* request_dat, uint32_t request_size, uint8_t* response_dat, uint32_t response_size) {
	//cleanup buffers
	serial_rpi.flushRX();
	serial_rpi.flushTX();

	//send the message
	serial_rpi.write(cmd);
	serial_rpi.write(request_size);
	serial_rpi.writeBuf(request_dat, request_size);
	uint16_t crc_request = calc_crc16(request_dat, request_size);
	serial_rpi.writeBuf((uint8_t*)&crc_request, sizeof(uint16_t));

	//receive the message. it is predefined how long the message length is
	uint32_t receive_message_size = response_size + 4;
	uint32_t time = 0;
	while (serial_rpi.available() > receive_message_size) {
		scheduler_task_sleep(1);
		time++;
		if (time > 200) {
			return 1; //error => header receive timeout.
		}
	}

	//read input buffer
	uint8_t rec_cmd = serial_rpi.read();
	if (rec_cmd != cmd) {
		return 2; //error => wrong command
	}
	uint8_t rec_size = serial_rpi.read();
	if (rec_size != response_size) {
		return 3; //error => wrong size
	}
	serial_rpi.readBuf(response_dat, response_size);
	uint16_t crc_response;
	serial_rpi.readBuf((uint8_t*)&crc_response, sizeof(uint16_t));
	uint16_t crc_calc = calc_crc16(request_dat, request_size);
	if (crc_response != crc_calc) {
		return 4; //error => crc wrong
	}

	return 0; //all ok
}


/**
 * Calculate CRC-16/CCITT-FALSE of the data.
 *
 *  Poly=0x1021, Init=0xFFFF, RefIn=False, RefOut=False, XorOut=0x0000
 *
 *  Args:
 *      data: input data
 *
 *  Returns:
 *      int: crc of the data
 */
static uint16_t calc_crc16(const uint8_t* data, uint32_t len) {
	uint16_t crc = 0xFFFF;
	for (uint32_t i = 0; i < len; i++) {
		uint8_t x = data[i];
		crc ^= x << 8;
		for (uint32_t j = 0; j < 8; j++) {
			if (crc & 0x8000) {
				crc = (crc * 2) ^ 0x1021;
			} else {
				crc = crc * 2;
			}
		}
	}
	return crc & 0xFFFF;
}
