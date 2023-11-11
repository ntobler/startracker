/*
 * tmc220x.h
 *
 *  Created on: Nov 11, 2023
 *      Author: ftobler
 */

#ifndef TMC220X_H_
#define TMC220X_H_


#include "stm32_hal.h"


class TMC220X {
private:
	int32_t _pos;
	void send_data(uint8_t reg, uint32_t data);
	void swuart_calcCRC(uint8_t* datagram, uint32_t datagramLength);
public:
	GPIO_TypeDef* en_port;
	uint16_t en_pin;
	GPIO_TypeDef* step_port;
	uint16_t step_pin;
	GPIO_TypeDef* dir_port;
	uint16_t dir_pin;
	UART_HandleTypeDef* uart;
	TMC220X();
	void send_init();
	void set_to(int32_t pos);
	void step_to(int32_t pos);
};



#endif /* TMC220X_H_ */
