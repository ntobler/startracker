/*
 * control.cpp
 *
 *  Created on: Nov 17, 2023
 *      Author: ftobler
 */


#include "control.h"
#include "stm32_hal.h"
#include "scheduler.h"
#include "application.h"
#include "string.h"
#include "main.h"
#include "rpi_protocol.h"


enum {
	RPI_ON_TIMEOUT = 60*10 * 3, //in 100ms ticks => 3min
	ADC_SAMPLE_LEN = 128,
	ADC_CHANNELS = 2,
};

static uint8_t do_capture = 0;
static uint8_t do_track = 0;
static uint8_t do_reset = 0;
static uint8_t do_shutdown = 0;
static uint32_t rpi_on_time = 0;
static uint32_t adc_dma_buf[ADC_SAMPLE_LEN * ADC_CHANNELS];
static float old_battery_voltage = 0.0f;
static float battery_charge_change = 0.0f;

Control_t control = {0};

static void measure_vbat();
static void rpi_power(uint32_t on);
static uint8_t battery_charge_level_state_machine(uint8_t state, uint32_t voltage);

extern ADC_HandleTypeDef hadc1;

void control_init() {
	do_capture = 0;
	do_track = 0;
	do_reset = 0;
	do_shutdown = 0;
	memset(&control, 0, sizeof(Control_t));

	HAL_GPIO_WritePin(POWER_ENABLE_GPIO_Port, POWER_ENABLE_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(MOTOR_BOOST_ENABLE_GPIO_Port, MOTOR_BOOST_ENABLE_Pin, GPIO_PIN_SET);

	HAL_ADC_Start_DMA(&hadc1, adc_dma_buf, ADC_SAMPLE_LEN * ADC_CHANNELS);
}


void control_update() {
	measure_vbat();

	if (old_battery_voltage == 0.0f) {
		old_battery_voltage = control.battery_voltage;
	}
	battery_charge_change = control.battery_voltage - old_battery_voltage;
	old_battery_voltage = old_battery_voltage * 0.998f + 0.002f * control.battery_voltage;

	//evaluate if charger is charging
//	control.charger_charging = HAL_GPIO_ReadPin(CHARGER_CHARGING_GPIO_Port, CHARGER_CHARGING_Pin) == GPIO_PIN_RESET;  //this does not work because pullop R10 is missing and power_enable is pulling the line low anyhow.
	control.charger_charging = control.battery_voltage > 4.00f;
	if (battery_charge_change > 0.02f) {
		control.charger_charging = 1;
	}
	if (battery_charge_change < -0.02f) {
		control.charger_charging = 0;
	}

	//evaluate if charger is done
//	control.charger_done = HAL_GPIO_ReadPin(CHARGER_DONE_GPIO_Port, CHARGER_DONE_Pin) == GPIO_PIN_RESET;
	control.charger_done = control.battery_voltage > 4.15f;
	if (battery_charge_change > 0.02f || battery_charge_change < -0.02f) {
		control.charger_done = 0;
	}

	control.charge_level = battery_charge_level_state_machine(control.charge_level, control.battery_voltage * 1000.0f);


	static uint32_t poll_timer = 0;
	const uint32_t POLL_TIME = 10;

	switch (control.state) {
	case CONTROL_IDLE:
		if (do_capture) {
			do_capture = 0;
			control.state = CONTROL_BOOTING;
		}
		break;
	case CONTROL_BOOTING:
		rpi_on_time = RPI_ON_TIMEOUT;
		poll_timer++;
		if (poll_timer > POLL_TIME) {
			Status* status = rpi_status();
			if (status) {
				control.state = CONTROL_MEASURING;
			}
			poll_timer = 0;
		}
		break;
	case CONTROL_MEASURING:
		rpi_on_time = RPI_ON_TIMEOUT;
		break;
	case CONTROL_RUNNING:
		break;
	default:
		control.state = CONTROL_IDLE;
	}

	//switch rpi on, off with a delay
	if (rpi_on_time) {
		rpi_on_time--;
	}
	rpi_power(rpi_on_time);

}


void control_action_capture() {
	do_capture = 1;
}

void control_action_track() {
	do_track = 1;
}

void control_action_reset() {
	do_reset = 1;
}

void control_action_shutdown() {
	do_shutdown = 1;
}





static void measure_vbat() {
//INTEGER variant
	//build average ADC voltage for battery. There is a LDO of 3.3V as the ADC reference.
	//There is also a 10k/10k resistive divider => factor 2
	uint32_t sum = 0;
	uint32_t count = 0;
	for (uint32_t i = 0; i < ADC_SAMPLE_LEN; i++) {
		uint32_t adc_bat = adc_dma_buf[i * ADC_CHANNELS + 0];
		uint32_t adc_vrefint = adc_dma_buf[i * ADC_CHANNELS + 1];
		uint32_t vrefint = __LL_ADC_CALC_VREFANALOG_VOLTAGE(adc_vrefint, LL_ADC_RESOLUTION_12B);
		uint32_t vbat = vrefint * adc_bat * 2 / 4096;
		if (vbat < 6000 && vbat > 2000) {
			sum += vbat;
			count++;
		}
	}
	float new_vbat = (float)sum / count / 1000;
	if (control.battery_voltage == 0.0f) {
		control.battery_voltage = new_vbat;
	} else {
		control.battery_voltage = control.battery_voltage * 0.90f + new_vbat * 0.10f;
	}

//FLOAT variant
//	//build average ADC voltage for battery. There is a LDO of 3.3V as the ADC reference.
//	//There is also a 10k/10k resistive divider => factor 2
//	float sum = 0;
//	uint32_t count = 0;
//	for (uint32_t i = 0; i < ADC_SAMPLE_LEN; i++) {
//		uint32_t adc_bat = adc_dma_buf[i * ADC_CHANNELS + 0];
//		uint32_t adc_vrefint = adc_dma_buf[i * ADC_CHANNELS + 1];
//		uint32_t vrefint = __LL_ADC_CALC_VREFANALOG_VOLTAGE(adc_vrefint,
//				LL_ADC_RESOLUTION_12B);
//		//float voltage = __LL_ADC_CALC_DATA_TO_VOLTAGE(vrefint, adc_bat, LL_ADC_RESOLUTION_12B) / 1000.0f * 2.0f;
//		float vbat = (vrefint / 1000.0f) * (adc_bat / 4096.0f) * 2.0f;
//		if (vbat < 6.0f && vbat > 2.0f) {
//			sum += vbat;
//			count++;
//		}
//	}
//	float new_vbat = sum / count;
//	if (control.battery_voltage == 0.0f) {
//		control.battery_voltage = new_vbat;
//	} else {
//		control.battery_voltage = control.battery_voltage * 0.8f
//				+ new_vbat * 0.2f;
//	}
}



static uint8_t battery_charge_level_state_machine(uint8_t state, uint32_t voltage) {
	switch (state) {
	case 0:
		if (voltage > 3451) return state + 1;
		return state;
	case 1:
		if (voltage < 3395) return state - 1;
		if (voltage > 3529) return state + 1;
		return state;
	case 2:
		if (voltage < 3515) return state - 1;
		if (voltage > 3577) return state + 1;
		return state;
	case 3:
		if (voltage < 3559) return state - 1;
		if (voltage > 3608) return state + 1;
		return state;
	case 4:
		if (voltage < 3600) return state - 1;
		if (voltage > 3654) return state + 1;
		return state;
	case 5:
		if (voltage < 3638) return state - 1;
		if (voltage > 3702) return state + 1;
		return state;
	case 6:
		if (voltage < 3692) return state - 1;
		if (voltage > 3749) return state + 1;
		return state;
	case 7:
		if (voltage < 3736) return state - 1;
		if (voltage > 3814) return state + 1;
		return state;
	case 8:
		if (voltage < 3797) return state - 1;
		if (voltage > 3888) return state + 1;
		return state;
	case 9:
		if (voltage < 3866) return state - 1;
		if (voltage > 3969) return state + 1;
		return state;
	case 10:
		if (voltage < 3941) return state - 1;
		return state;
	default:
		return 5;
	}
}


static void rpi_power(uint32_t on) {
	control.rpi_running = on;
	HAL_GPIO_WritePin(POWER_ENABLE_GPIO_Port, POWER_ENABLE_Pin, (GPIO_PinState)on);
}

