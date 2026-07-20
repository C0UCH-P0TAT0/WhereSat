/**
 * @file mekf.h
 * @brief 3-State Multiplicative Extended Kalman Filter.
 *
 * Estimates attitude error only (gyro bias removed).
 *
 * @author Aditya (WhereSat Team)
 */

#ifndef INC_MEKF_H_
#define INC_MEKF_H_

#include "quaternion.h"
#include <stdbool.h>

typedef struct {
    Quaternion_t q;          // Estimated attitude (Inertial to Body)
    float P[3][3];           // Covariance matrix (3x3 for attitude error)
    float Q_v;               // Gyro noise variance
} MEKF_t;

void mekf_init(MEKF_t *f, Quaternion_t initial_q);
void mekf_predict(MEKF_t *f, Vector3_t gyro_meas, float dt);
void mekf_update(MEKF_t *f, Quaternion_t q_meas, float r_noise);

#endif