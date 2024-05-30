/*
 * Serialh
 *
 *  Created on: 17.10.2020
 *      Author: Florin
 */

#ifndef SERIAL_VCP_H_
#define SERIAL_VCP_H_

#include "stdint.h"
#include "serial.h"

#ifdef __cplusplus

#define OUTBUFSIZE 128

//return 0 on success
typedef uint32_t (*send_cb_t)(uint8_t* buf, uint32_t len);

/**
 * UART implementation for use with Virtual COM port.
 * the implementation uses a send and receive FIFO buffer
 */
class SerialVCP {
private:
	Buffer in;
	Buffer out;
	uint32_t outbuf_size;
	uint8_t outbuf[OUTBUFSIZE];
	send_cb_t vpc_send_cb;
	inline void enableTx();
public:
	SerialVCP(send_cb_t _vpc_send_cb);
	uint32_t available();
	void flushRX();
	void flushTX();
	void print(const char* str);
	uint8_t read();
	uint32_t readBuf(uint8_t* buf, uint16_t len);
	void writeBuf(const uint8_t* buf, uint16_t len);
	void write(const uint8_t data);
	void vcp_receive_cb(uint8_t* buf, uint32_t len);
	void vcp_send_cmplt_cb();
};

#endif


#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif




#endif /* SERIAL_VCP_H_ */
