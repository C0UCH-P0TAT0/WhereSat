#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include "main.h"
#include "quaternion.h"
#include <stdbool.h> // <--- ADD THIS for bool

#define TELEM_SOF 0xAA

HAL_StatusTypeDef telemetry_send(
    const Quaternion_t *q,
    const Vector3_t *omega,
    const Vector3_t *torque,
    bool locked,
    uint8_t num_stars
);

#endif
