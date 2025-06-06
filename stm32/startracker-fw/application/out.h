//Autogenerated code

#ifndef OUT_H_
#define OUT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct __attribute__((__packed__)) {
    float x; // x part of the quaternion
    float y; // y part of the quaternion
    float z; // z part of the quaternion
    float w; // w part of the quaternion
} Quaternion;

typedef enum AttitudeEstimationMode : uint8_t {
    OFF = 0,
    ON = 1,
    SINGLE = 2,
    RECORD_DARKFRAME = 3,
} AttitudeEstimationMode;

typedef enum AcknowledgeEnum : uint8_t {
    NACK = 0,
    ACK = 1,
} AcknowledgeEnum;

typedef struct __attribute__((__packed__)) {
} EmptyMessage;

typedef struct __attribute__((__packed__)) {
    AttitudeEstimationMode attitude_estimation_mode;
    uint16_t current_number_of_matches;
    uint16_t average_number_of_matches;
    Quaternion quaternion; // curent quaternion
    uint16_t estimation_id; // Id of the current attitude estimation sample
} Status;

typedef struct __attribute__((__packed__)) {
    float start; // Start time in seconds
    float stop; // Stop time in seconds
    float coeffs[3][4]; // Polynomial coefficients
} Trajectory;

typedef struct __attribute__((__packed__)) {
    uint8_t min_matches; // Min matches required in the attitude estimation
    uint8_t attitude_estimation_timeout_ms;
    uint16_t exposure_ms;
    uint8_t gain;
} Settings;

typedef struct __attribute__((__packed__)) {
    AcknowledgeEnum ack; // 1 if acknowledge okay else 0
} Acknowledge;

#ifdef __cplusplus
}
#endif

#endif /* OUT_H_ */
