/*
 * imu.cpp
 *
 *  Created on: Nov 14, 2023
 *      Author: ftobler
 */


#include "imu.h"
#include "mpu9250.h"
#include "string.h"
#include "stm32_hal.h"


static MPU9250 mpu;
extern TIM_HandleTypeDef htim5;   //microseconds counter up to 4294967295

Imu_data_t imu;


void imu_init() {
	//	uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
	mpu.resetMPU9250(); // Reset registers to default in preparation for device calibration
	mpu.calibrateMPU9250(imu.gyro_bias, imu.accel_bias); // Calibrate gyro and accelerometers, load biases in bias registers
	mpu.initMPU9250();
    mpu.initAK8963(imu.mag_calibration);
}


void imu_update() {
	mpu.readAccelData(imu.raw.a);  // Read the x/y/z adc values
	float aRes = mpu.getAres();
    // Now we'll calculate the accleration value into actual g's
    imu.meas.a[0] = (float)imu.raw.a[0]*aRes;// - imu.accel_bias[0];  // get actual g value, this depends on scale being set
    imu.meas.a[1] = (float)imu.raw.a[1]*aRes;// - imu.accel_bias[1];
    imu.meas.a[2] = (float)imu.raw.a[2]*aRes;// - imu.accel_bias[2];

	mpu.readGyroData(imu.raw.g);  // Read the x/y/z adc values
	float gRes = mpu.getGres();
    // Calculate the gyro value into actual degrees per second
	imu.meas.g[0] = (float)imu.raw.g[0]*gRes - imu.gyro_bias[0];  // get actual gyro value, this depends on scale being set
    imu.meas.g[1] = (float)imu.raw.g[1]*gRes - imu.gyro_bias[1];
    imu.meas.g[2] = (float)imu.raw.g[2]*gRes - imu.gyro_bias[2];

	mpu.readMagData(imu.raw.m);  // Read the x/y/z adc values
	float mRes = mpu.getMres();
	float magbias[3] = {0};
//    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
//    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
//    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    imu.meas.m[0] = (float)imu.raw.m[0]*mRes*imu.mag_calibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
    imu.meas.m[1] = (float)imu.raw.m[1]*mRes*imu.mag_calibration[1] - magbias[1];
    imu.meas.m[2] = (float)imu.raw.m[2]*mRes*imu.mag_calibration[2] - magbias[2];

    static uint32_t last_micros = 0;
    uint32_t micros = htim5.Instance->CNT;
    float deltat = ((micros - last_micros)/1000000.0f); // set integration time by time elapsed since last filter update
    last_micros = micros;

    const float PI = 3.14159265358979323846f;
	mpu.MadgwickQuaternionUpdate(
			imu.meas.a[0], imu.meas.a[1], imu.meas.a[2],
			imu.meas.g[0]*PI/180.0f, imu.meas.g[1]*PI/180.0f, imu.meas.g[2]*PI/180.0f,
			imu.meas.m[0], imu.meas.m[1], imu.meas.m[2],
			deltat);
	memcpy(imu.quaternion, mpu.q, sizeof(float) * 4);


    imu.raw.temp = mpu.readTempData();  // Read the adc values
    imu.meas.temp = ((float) imu.raw.temp) / 333.87f + 21.0f; // Temperature in degrees Centigrade

    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    float* q = imu.quaternion;
	float yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	float pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	float roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	imu.fusion.pitch = pitch * 180.0f / PI;
	imu.fusion.yaw   = (yaw * 180.0f / PI) - 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04;
	imu.fusion.roll  = roll * 180.0f / PI;
}

