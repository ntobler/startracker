/*
 * gyro.h
 *
 *  Created on: Sep 17, 2025
 *      Author: ntobler
 */

#ifndef GYRO_H_
#define GYRO_H_

#include <array>
#include <cmath>
#include <numbers>

#include "arm_math.h"
#include "stdint.h"
#include "stm32_hal.h"

constexpr float RAD2DEG = 180.0 / std::numbers::pi;

enum {
    N_ELEMENTS = 16,
};

struct __packed DataPayload {
    uint8_t dummy;
    uint8_t address;
    uint8_t temperature_higher;
    uint8_t temperature_lower;
    uint8_t accel_x_higher;
    uint8_t accel_x_lower;
    uint8_t accel_y_higher;
    uint8_t accel_y_lower;
    uint8_t accel_z_higher;
    uint8_t accel_z_lower;
    uint8_t gyro_x_higher;
    uint8_t gyro_x_lower;
    uint8_t gyro_y_higher;
    uint8_t gyro_y_lower;
    uint8_t gyro_z_higher;
    uint8_t gyro_z_lower;
};

class HpFilter {
   private:
    float alpha_;
    float y_1_;
    float x_1_;

   public:
    HpFilter(float alpha) : alpha_{alpha}, y_1_{0}, x_1_{0} {}

    float process(float x) {
        float res = (y_1_ + x - x_1_) * alpha_;
        y_1_ = res;
        x_1_ = x;
        return res;
    }
};

class Quaternion {
   public:
    float q_[4];

    Quaternion() : q_{0.0, 0.0, 0.0, 1.0} {};
    Quaternion(float x, float y, float z, float w) : q_{x, y, z, w} {};

    float x() const { return q_[0]; }
    float y() const { return q_[1]; }
    float z() const { return q_[2]; }
    float w() const { return q_[3]; }

    void normalize() { arm_quaternion_normalize_f32(q_, q_, 1); }

    static Quaternion from_angular_velocities(float x, float y, float z, float delta_t) {
        float norm = x * x + y * y + z * z;
        arm_sqrt_f32(norm, &norm);
        float s, c;
        arm_sin_cos_f32(norm * delta_t * 0.5 * RAD2DEG, &s, &c);
        float inv_norm = 1.0 / norm;
        return Quaternion{
            x * inv_norm * s,
            y * inv_norm * s,
            z * inv_norm * s,
            c,
        };
    }

    void multiply_right(Quaternion &q_right) {
        arm_quaternion_product_single_f32(q_, q_right.q_, q_);
    }
    void multiply_left(Quaternion &q_left) { arm_quaternion_product_single_f32(q_left.q_, q_, q_); }
};

class Gyro {
   private:
    std::array<DataPayload, N_ELEMENTS> buffer_;
    volatile size_t write_index_;
    size_t read_index_;
    SPI_HandleTypeDef *hspi_;

    std::array<HpFilter, 3> bias_filters_;
    Quaternion q_;

   public:
    Gyro(SPI_HandleTypeDef *hspi);
    void tick();
    void fetch_next();
    void spi_tx_rx_complete_callback(SPI_HandleTypeDef *hspi);
};

#endif /* GYRO_H_ */
