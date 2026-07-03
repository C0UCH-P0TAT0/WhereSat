/**
 * @file quaternion.c
 * @brief Implementation of quaternion math routines.
 *
 * @author Aditya (WhereSat Team)
 */

#include "quaternion.h"
#include <math.h>

Quaternion_t quat_normalize(Quaternion_t q) {
    float mag = sqrtf(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
    if (mag > 0.00001f) {
        q.q0 /= mag; q.q1 /= mag; q.q2 /= mag; q.q3 /= mag;
    } else {
        q.q0 = 1.0f; q.q1 = 0.0f; q.q2 = 0.0f; q.q3 = 0.0f;
    }
    return q;
}

Quaternion_t quat_multiply(Quaternion_t q, Quaternion_t p) {
    Quaternion_t r;
    r.q0 = q.q0*p.q0 - q.q1*p.q1 - q.q2*p.q2 - q.q3*p.q3;
    r.q1 = q.q0*p.q1 + q.q1*p.q0 + q.q2*p.q3 - q.q3*p.q2;
    r.q2 = q.q0*p.q2 - q.q1*p.q3 + q.q2*p.q0 + q.q3*p.q1;
    r.q3 = q.q0*p.q3 + q.q1*p.q2 - q.q2*p.q1 + q.q3*p.q0;
    return r;
}

Quaternion_t quat_conjugate(Quaternion_t q) {
    return (Quaternion_t){q.q0, -q.q1, -q.q2, -q.q3};
}

Vector3_t quat_rotate_vector(Quaternion_t q, Vector3_t v) {
    Quaternion_t v_quat = {0, v.x, v.y, v.z};
    Quaternion_t q_conj = quat_conjugate(q);
    Quaternion_t rotated = quat_multiply(quat_multiply(q, v_quat), q_conj);
    return (Vector3_t){rotated.q1, rotated.q2, rotated.q3};
}

void quat_to_dcm(Quaternion_t q, float dcm[3][3]) {
    float q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
    dcm[0][0] = 1 - 2*(q2*q2 + q3*q3);
    dcm[0][1] = 2*(q1*q2 - q0*q3);
    dcm[0][2] = 2*(q1*q3 + q0*q2);
    dcm[1][0] = 2*(q1*q2 + q0*q3);
    dcm[1][1] = 1 - 2*(q1*q1 + q3*q3);
    dcm[1][2] = 2*(q2*q3 - q0*q1);
    dcm[2][0] = 2*(q1*q3 - q0*q2);
    dcm[2][1] = 2*(q2*q3 + q0*q1);
    dcm[2][2] = 1 - 2*(q1*q1 + q2*q2);
}
