/**
 * @file controller.c
 * @brief PD Attitude Controller (Scalar-Last).
 * @author Aditya (WhereSat Team)
 */

#include "controller.h"

#define MAX_TORQUE 0.02f

Vector3_t controller_compute_torque(PD_Controller_t *c, Quaternion_t q_curr, Quaternion_t q_targ, Vector3_t omega) {
    // 1. Error Quaternion (q_curr_inv * q_targ)
    Quaternion_t q_err = quat_multiply(quat_conjugate(q_curr), q_targ);

    // Shortest path check (Scalar is .w)
    float s = (q_err.w >= 0) ? 1.0f : -1.0f;

    // 2. PD Law: T = 2*Kp*q_vec - Kd*omega
    Vector3_t t;
    t.x = ( c->Kp * (s * q_err.x)) - (c->Kd * omega.x);
    t.y = ( c->Kp * (s * q_err.y)) - (c->Kd * omega.y);
    t.z = ( c->Kp * (s * q_err.z)) - (c->Kd * omega.z);

    // 3. Per-Axis Saturation
    if (t.x >  MAX_TORQUE) t.x =  MAX_TORQUE;
    if (t.x < -MAX_TORQUE) t.x = -MAX_TORQUE;
    if (t.y >  MAX_TORQUE) t.y =  MAX_TORQUE;
    if (t.y < -MAX_TORQUE) t.y = -MAX_TORQUE;
    if (t.z >  MAX_TORQUE) t.z =  MAX_TORQUE;
    if (t.z < -MAX_TORQUE) t.z = -MAX_TORQUE;

    return t;
}
