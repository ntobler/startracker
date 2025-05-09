/*
 * mpu9250.h
 *
 *  Created on: Nov 14, 2023
 *      Author: ftobler
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "mpu9250_hal.h"
#include "math.h"

enum Ascale {
	AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G
};


enum Gscale {
	GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
};

enum Mscale {
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
};


class MPU9250 {

protected:
	uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
	uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
	uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
	uint8_t Mmode = 0x06; // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR

	//Set up I2C, (SDA,SCL)
	I2C i2c;

	// Pin definitions
//	int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

//	int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
//	int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
//	int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
//	float gyroBias[3] = { 0, 0, 0 }, accelBias[3] = { 0, 0, 0 }; // Bias corrections for gyro and accelerometer
//	float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
//	int16_t tempCount; // Stores the real internal chip temperature in degrees Celsius
//	float temperature;
	float SelfTest[6];

//	int delt_t = 0; // used to control display output rate
//	int count = 0;  // used to control display output rate

	// parameters for 6 DoF sensor fusion calculations
	float PI = 3.14159265358979323846f;
	float GyroMeasError = PI * (60.0f / 180.0f); // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
//	float GyroMeasDrift = PI * (1.0f / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
//	float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
	#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
	#define Ki 0.0f

//	float pitch, yaw, roll;
//	float deltat = 0.0f;             // integration interval for both filter schemes
//	int lastUpdate = 0, firstUpdate = 0, Now = 0; // used to calculate integration interval                               // used to calculate integration interval

	float eInt[3] = { 0.0f, 0.0f, 0.0f }; // vector to hold integral error for Mahony method
//	float magCalibration[3] = { 0, 0, 0 };
//	float magbias[3] = { 0, 0, 0 }; // Factory mag calibration and mag bias


	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
	char readByte(uint8_t address, uint8_t subAddress);
	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

public:
	float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };           // vector to hold quaternion
//===================================================================================================================
//====== Set of useful function to access acceleratio, gyroscope, and temperature data
//===================================================================================================================


	float getMres();
	float getGres();
	float getAres();

	void readAccelData(int16_t * destination);
	void readGyroData(int16_t * destination);
	void readMagData(int16_t * destination);

	int16_t readTempData();
	void resetMPU9250();
	void initAK8963(float * destination);
	void initMPU9250();

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
	void calibrateMPU9250(float * dest1, float* dest2);

// Accelerometer and gyroscope self test; check calibration wrt factory settings
	void MPU9250SelfTest(float * destination);

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
	void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

	// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
	// measured ones.
	void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
};

#ifdef __cplusplus
}
#endif

#endif /* MPU9250_H_ */
