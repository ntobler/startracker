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
#include "quat.h"
#include "stdint.h"
#include "stm32_hal.h"

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

class LpIntFilter {
   private:
    float alpha_;
    float y_1_;

   public:
    LpIntFilter(float alpha) : alpha_{alpha}, y_1_{0} {}

    float process(float x) {
        float res = x + alpha_ * y_1_;
        y_1_ = res;
        return res;
    }
};

class Gyro {
   private:
    bool started_;
    std::array<DataPayload, N_ELEMENTS> buffer_;
    volatile size_t write_index_;
    size_t read_index_;
    SPI_HandleTypeDef *hspi_;

    std::array<HpFilter, 3> bias_filters_;
    Vec3 gyro_;
    Vec3 accel_;
    Quaternion q_;
    std::array<LpIntFilter, 3> pos_filters_;
    Vec3 pos_;
    uint16_t id_;

    void write(uint8_t address, const uint8_t *data, size_t len);

   public:
    Gyro(SPI_HandleTypeDef *hspi);
    void start();
    void tick();
    void get_xy_images(float &x, float &y);
    const Vec3 &get_gyro() const { return gyro_; };
    const Vec3 &get_accel() const { return accel_; };
    const Vec3 &get_pos() const { return pos_; };
    const uint16_t get_id() const { return id_; };
    const Quaternion &get_quat() const { return q_; };
    void set_quat(Quaternion &q) { q_ = q; }
    void fetch_next();
    void spi_tx_rx_complete_callback(SPI_HandleTypeDef *hspi);
};

#endif /* GYRO_H_ */
