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


Control_t control = {0};

static void measure_vbat();
static void rpi_power(uint32_t on);
static uint32_t rpi_responsive();

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

	//evaluate if charger is charging
//	control.charger_charging = HAL_GPIO_ReadPin(CHARGER_CHARGING_GPIO_Port, CHARGER_CHARGING_Pin) == GPIO_PIN_RESET;  //this does not work because pullop R10 is missing and power_enable is pulling the line low anyhow.
	control.charger_charging = control.battery_voltage > 4.00f;

	//evaluate if charger is done
//	control.charger_done = HAL_GPIO_ReadPin(CHARGER_DONE_GPIO_Port, CHARGER_DONE_Pin) == GPIO_PIN_RESET;
	control.charger_done = control.battery_voltage > 4.20f;




	switch (control.state) {
	case CONTROL_IDLE:
		if (do_capture) {
			do_capture = 0;
			control.state = CONTROL_BOOTING;
		}
		break;
	case CONTROL_BOOTING:
		rpi_on_time = RPI_ON_TIMEOUT;
		if (rpi_responsive()) {
			control.state = CONTROL_MEASURING;
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
	//build average ADC voltage for battery. There is a LDO of 3.3V as the ADC reference.
	//There is also a 10k/10k resistive divider => factor 2
	float sum = 0;
	uint32_t count = 0;
	for (uint32_t i = 0; i < ADC_SAMPLE_LEN; i++) {
		uint32_t adc_bat = adc_dma_buf[i * ADC_CHANNELS + 0];
		uint32_t adc_vrefint = adc_dma_buf[i * ADC_CHANNELS + 1];
		uint32_t vrefint = __LL_ADC_CALC_VREFANALOG_VOLTAGE(adc_vrefint,
				LL_ADC_RESOLUTION_12B);
		//float voltage = __LL_ADC_CALC_DATA_TO_VOLTAGE(vrefint, adc_bat, LL_ADC_RESOLUTION_12B) / 1000.0f * 2.0f;
		float vbat = (vrefint / 1000.0f) * (adc_bat / 4096.0f) * 2.0f;
		if (vbat < 6.0f && vbat > 2.0f) {
			sum += vbat;
			count++;
		}
	}
	float new_vbat = sum / count;
	if (control.battery_voltage == 0.0f) {
		control.battery_voltage = new_vbat;
	} else {
		control.battery_voltage = control.battery_voltage * 0.8f
				+ new_vbat * 0.2f;
	}
}

static void rpi_power(uint32_t on) {
	//TODO
	control.rpi_running = 1;
}

static uint32_t rpi_responsive() {
	return 1;
}
