/*
 * control.h
 *
 *  Created on: Nov 17, 2023
 *      Author: ftobler
 */

#ifndef CONTROL_H_
#define CONTROL_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stdint.h"


typedef enum {
	CONTROL_RPI_IDLE,
	CONTROL_RPI_BOOTING,
	CONTROL_RPI_READY,
	CONTROL_RPI_SHUTDOWN,
} Control_rpi_state;

typedef enum {
	RPI_DO_BOOT = 0x01,
	RPI_DO_SHUTDOWN = 0x02,
	RPI_DO_CALC = 0x04,
} Control_rpi_flags_t;

typedef struct {
	float battery_voltage;
	uint8_t charger_charging;
	uint8_t charger_done;
	Control_rpi_state state;
	Control_rpi_flags_t flags;
	uint8_t charge_level;
} Control_t;


void control_init();
void control_update();
void control_action_start();
void control_action_shutdown();


#ifdef __cplusplus
}
#endif


#endif /* CONTROL_H_ */
