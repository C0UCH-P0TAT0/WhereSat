/**
 * @file controller.h
 * @brief Attitude PD Controller for reaction wheel torque.
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "quaternion.h"

typedef struct {
    float Kp;
    float Kd;
    float max_torque; // Saturation limit (Nm)
} PD_Controller_t;

Vector3_t controller_compute_torque(PD_Controller_t *c, Quaternion_t q_curr, Quaternion_t q_targ, Vector3_t omega);

#endif
