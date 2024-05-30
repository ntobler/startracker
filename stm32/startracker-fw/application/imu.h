/*
 * imu.h
 *
 *  Created on: Nov 14, 2023
 *      Author: ftobler
 */

#ifndef IMU_H_
#define IMU_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

typedef struct {
	int16_t a[3];
	int16_t g[3];
	int16_t m[3];
	int16_t temp;
} Raw_t;

typedef struct {
	float a[3];
	float g[3];
	float m[3];
	float temp;
} Meas_t;

typedef struct {
	float yaw, pitch, roll;
} Fusion_t;

typedef struct {
	float gyro_bias[3];
	float accel_bias[3];
	float mag_calibration[3];
	Raw_t raw;
	Meas_t meas;
	float quaternion[4];
	Fusion_t fusion;
} Imu_data_t;


void imu_init();
void imu_update();


#ifdef __cplusplus
}
#endif


#endif /* IMU_H_ */
