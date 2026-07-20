/**
 * @file telemetry.h
 * @brief Telemetry definitions (Bias Removed)
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include "main.h"
#include "quaternion.h"
#include <stdbool.h>

#define TELEM_SOF 0xAA

HAL_StatusTypeDef telemetry_send(
    const Quaternion_t *q_est,
    const Vector3_t *w_est,
    const Vector3_t *torque,
    const Quaternion_t *q_quest,
    const Vector3_t *gyro_meas,
    float innovation_dot,
    bool locked,
    uint8_t num_stars
);

#endif