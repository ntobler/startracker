/*
 * rpi.h
 *
 *  Created on: Sep 24, 2025
 *      Author: ntobler
 */

#ifndef RPI_H_
#define RPI_H_

#include <stdio.h>

#include <cstring>

#include "ledring.h"
#include "main.h"
#include "quat.h"
#include "serial.h"

constexpr size_t ESTIMATED_STARTUP_TIME_MS = 15000;
constexpr size_t ESTIMATED_SHUTDOWN_TIME_MS = 10000;

class Rpi {
   private:
    bool power_;
    Gyro* gyro_;
    Serial* rpi_serial_;
    LedRing* led_ring_;
    uint32_t state_time_ms_;

    enum State {
        OFF,
        BOOT_RPI,
        NO_MATCH,
        MATCH,
        SHUTDOWN,
    };
    State state_;

    void change_state(State new_state) {
        state_ = new_state;
        state_time_ms_ = 0;
        switch (state_) {
            case State::OFF:
                power_off();
                led_ring_->set_mode(LedMode::OFF);
                break;
            case State::BOOT_RPI:
                power_on();
                led_ring_->set_progress(0.0);
                led_ring_->set_mode(LedMode::PROGRESS);
                break;
            case State::NO_MATCH:
                led_ring_->set_mode(LedMode::SEARCH);
                break;
            case State::MATCH:
                led_ring_->set_mode(LedMode::LEVEL_XY2);
                break;
            case State::SHUTDOWN:
                led_ring_->set_progress(1.0);
                led_ring_->set_mode(LedMode::PROGRESS);
                send_shutdown_request();
                break;
            default:
                break;
        }
    }

   public:
    Rpi(Gyro* gyro, Serial* rpi_serial, LedRing* led_ring)
        : power_{0},
          gyro_{gyro},
          rpi_serial_{rpi_serial},
          led_ring_{led_ring},
          state_time_ms_{0},
          state_{State::OFF} {
        HAL_GPIO_WritePin(RPI_ENABLE_GPIO_Port, RPI_ENABLE_Pin, GPIO_PIN_RESET);
    };
    void start() { change_state(State::OFF); }
    void power_on() {
        HAL_GPIO_WritePin(RPI_ENABLE_GPIO_Port, RPI_ENABLE_Pin, GPIO_PIN_SET);
        power_ = true;
    };
    void power_off() {
        HAL_GPIO_WritePin(RPI_ENABLE_GPIO_Port, RPI_ENABLE_Pin, GPIO_PIN_RESET);
        power_ = false;
    };
    void send_quat(const Quaternion& quat, uint16_t id) {
        uint8_t buffer[18];
        *((float*)&buffer[0]) = quat.w();
        *((float*)&buffer[4]) = quat.x();
        *((float*)&buffer[8]) = quat.y();
        *((float*)&buffer[12]) = quat.z();
        *((uint16_t*)&buffer[16]) = id;
        rpi_serial_->write_command(QUAT, buffer, sizeof(quat.q_) + 2);
    };
    void send_shutdown_request() {
        uint8_t payload = 31;
        rpi_serial_->write_command(SHUT_DOWN_REQUEST, &payload, 1);
    };

    void tick_10ms() {
        state_time_ms_ += 10;

        switch (state_) {
            case State::OFF:
                break;
            case State::BOOT_RPI: {
                // TODO replace with check for response and add timeout
                if (state_time_ms_ > ESTIMATED_STARTUP_TIME_MS) {
                    change_state(State::NO_MATCH);
                }
                float p = ((float)state_time_ms_) * (1.0f / ESTIMATED_STARTUP_TIME_MS);
                led_ring_->set_progress(1.0f - std::exp(-p * 4.0f));
            } break;
            case State::NO_MATCH:
                break;
            case State::MATCH:
                break;
            case State::SHUTDOWN: {
                if (state_time_ms_ > ESTIMATED_SHUTDOWN_TIME_MS) {
                    change_state(State::OFF);
                }
                float p = ((float)state_time_ms_) * (1.0f / ESTIMATED_SHUTDOWN_TIME_MS);
                led_ring_->set_progress(std::exp(-p * 4.0f));
                send_shutdown_request();
            } break;
            default:
                break;
        }
    }

    void on_quat() {
        state_time_ms_ += 10;

        switch (state_) {
            case State::OFF:
                break;
            case State::BOOT_RPI: {
            } break;
            case State::NO_MATCH:
                send_quat(gyro_->get_quat(), gyro_->get_id());
                break;
            case State::MATCH:
                send_quat(gyro_->get_quat(), gyro_->get_id());
                break;
            case State::SHUTDOWN: {
            } break;
            default:
                break;
        }
    }

    void star_quat(Quaternion& q_req, uint16_t id) {
        if (q_req.is_non_zero()) {
            Quaternion q = gyro_->get_quat().inv();
            q.multiply_left(q_req);
            gyro_->set_correction_quat(q);
        } else {
            __NOP();
        }
        switch (state_) {
            case State::OFF:
                break;
            case State::NO_MATCH:
                change_state(State::MATCH);
                break;
            case State::MATCH:
                break;
            case State::SHUTDOWN:
                break;
            default:
                break;
        }
    }

    void short_click() {
        switch (state_) {
            case State::OFF:
                change_state(State::BOOT_RPI);
                break;
            case State::NO_MATCH:
                break;
            case State::MATCH:
                break;
            case State::SHUTDOWN:
                break;
            default:
                break;
        }
    }

    void long_press() {
        switch (state_) {
            case State::OFF:
                break;
            case State::BOOT_RPI:
                break;
            case State::NO_MATCH:
                change_state(State::SHUTDOWN);
                break;
            case State::MATCH:
                change_state(State::SHUTDOWN);
                break;
            case State::SHUTDOWN:
                break;
            default:
                break;
        }
    }
};

#endif /* RPI_H_ */
