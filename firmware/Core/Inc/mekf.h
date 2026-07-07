/**
 * @file mekf.h
 * @brief 6-State Multiplicative Extended Kalman Filter.
 *
 * Estimates attitude error and gyro bias. Uses a 100Hz prediction loop
 * and a variable-rate update loop (Star Tracker).
 *
 * @author Aditya (WhereSat Team)
 */

#ifndef INC_MEKF_H_
#define INC_MEKF_H_

#include "quaternion.h"
#include <stdbool.h>

typedef struct {
    Quaternion_t q;          // Estimated attitude (Inertial to Body)
    float beta[3];           // Gyro bias estimate [rad/s]
    float P[6][6];           // Covariance matrix
    float Q_v;               // Gyro noise variance
    float Q_u;               // Bias random walk variance
} MEKF_t;

void mekf_init(MEKF_t *f, Quaternion_t initial_q);
void mekf_predict(MEKF_t *f, Vector3_t gyro_meas, float dt);
void mekf_update(MEKF_t *f, Quaternion_t q_meas, float r_noise);

#endif
