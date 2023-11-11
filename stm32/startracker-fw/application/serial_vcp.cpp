/*
 * serial.cpp
 *
 *  Created on: Feb 3, 2022
 *      Author: cchtofl01
 */

#include "serial_vcp.h"
#include "string.h"
#include "usbd_cdc.h"



extern USBD_HandleTypeDef hUsbDeviceFS;

/**
 * constructor. Make sure huart pointer is set to 0
 * so errors can be handled
 */
SerialVCP::SerialVCP(send_cb_t _vpc_send_cb) {
	vpc_send_cb = _vpc_send_cb;
	in.init();
	outbuf_size = 0;
}

uint32_t SerialVCP::available(){
	return in.getAvailable();
}


void SerialVCP::flushRX(){
	in.init();
}


void SerialVCP::flushTX(){

}


void SerialVCP::print(const char* str) {
	int len = strlen(str);
	writeBuf((const uint8_t*)str, len);
}


/**
 * unguarded about overflow!
 * use Serial_available!
 */
uint8_t SerialVCP::read() {
	return in.getByte();
}


uint32_t SerialVCP::readBuf(uint8_t* buf, uint16_t len) {
	uint16_t count = 0;
	while (in.getAvailable() && (count < len)) {
		buf[count] = in.getByte();
		count++;
	}
	return count;
}


int wait_counter1 = 0;
int wait_counter2 = 0;

void SerialVCP::writeBuf(const uint8_t* buf, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		out.setByte(buf[i]);
	}
	enableTx();
}



void SerialVCP::write(const uint8_t data) {
	out.setByte(data);
	enableTx();
}


void SerialVCP::vcp_receive_cb(uint8_t* buf, uint32_t len) {
	for (uint32_t i = 0; i < len; i++) {
		in.setByte(buf[i]);
	}
}

void SerialVCP::vcp_send_cmplt_cb() {
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
	if (hcdc->TxState == 0) {
		//ready to send
		uint32_t avail = out.getAvailable();
		if (avail) {
			for (uint32_t i = 0; i < avail; i++) {
				outbuf[i] = out.getByte();
			}
			outbuf_size = avail;
			vpc_send_cb(outbuf, avail);
		}
	}

//	int avail = out.getAvailable();
//	if (avail) {
//
//	}
//	if (outbuf_size) {
//		int ret = vpc_send_cb(outbuf, outbuf_size);
//		if (ret == 0) {
//			outbuf_size = 0;
//			memset(outbuf, 0, OUTBUFSIZE);
//		}
//	}


//	return;
//	if (outbuf_ready) {
//		//outbuf already filled with data. just send it.
//		if (vpc_send_cb(outbuf, outbuf_size) == 0) {
//			outbuf_ready  = 0; //success
//		}
//	} else {
//		//fill outbuf with available data and try to send it.
//		uint32_t available = out.getAvailable();
//		if (available) {
//			if (available > OUTBUFSIZE) {
//				available = OUTBUFSIZE;
//			}
//			for (uint32_t i = 0; i < available; i++) {
//				outbuf[i] = out.getByte();
//			}
//			if (vpc_send_cb(outbuf, available)) {
//				//failed to send outbuf
//				//mark it as pre-prepared for next time.
//				outbuf_ready = 1;
//				outbuf_size = available;
//			} else {
//				outbuf_ready  = 0; //success
//			}
//		}
//	}
}

inline void SerialVCP::enableTx() {
	vcp_send_cmplt_cb();
}
