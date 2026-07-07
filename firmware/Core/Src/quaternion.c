/**
 * @file quaternion.c
 * @brief Implementation of Scalar-Last [x, y, z, w] quaternion math.
 *
 * Provides the Hamilton product, conjugation, and rotation routines
 * specifically mapped to the scalar-last memory layout.
 *
 * @author Aditya (WhereSat Team)
 */

#include "quaternion.h"
#include <math.h>

/**
 * @brief Normalizes the quaternion to unit length.
 */
Quaternion_t quat_normalize(Quaternion_t q) {
    float mag = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);

    if (mag > 1e-6f) {
        q.x /= mag;
        q.y /= mag;
        q.z /= mag;
        q.w /= mag;
    } else {
        // Return identity if magnitude is near zero
        q.x = 0.0f; q.y = 0.0f; q.z = 0.0f; q.w = 1.0f;
    }
    return q;
}

/**
 * @brief Performs the Hamilton Product of two quaternions (Scalar-Last).
 * Formula: r = q * p
 */
Quaternion_t quat_multiply(Quaternion_t q, Quaternion_t p) {
    Quaternion_t r;

    r.x =  q.w * p.x + q.x * p.w + q.y * p.z - q.z * p.y;
    r.y =  q.w * p.y - q.x * p.z + q.y * p.w + q.z * p.x;
    r.z =  q.w * p.z + q.x * p.y - q.y * p.x + q.z * p.w;
    r.w =  q.w * p.w - q.x * p.x - q.y * p.y - q.z * p.z;

    return r;
}

/**
 * @brief Returns the conjugate of the quaternion (negates vector part).
 */
Quaternion_t quat_conjugate(Quaternion_t q) {
    return (Quaternion_t){-q.x, -q.y, -q.z, q.w};
}

/**
 * @brief Rotates a 3D vector using the quaternion: v' = q * v * q_conj
 */
Vector3_t quat_rotate_vector(Quaternion_t q, Vector3_t v) {
    // Convert vector to a pure quaternion (scalar w = 0)
    Quaternion_t v_quat = {v.x, v.y, v.z, 0.0f};

    Quaternion_t q_conj = quat_conjugate(q);

    // v' = q * v_quat * q_conj
    Quaternion_t rotated = quat_multiply(quat_multiply(q, v_quat), q_conj);

    return (Vector3_t){rotated.x, rotated.y, rotated.z};
}

/**
 * @brief Converts a quaternion to a 3x3 Direction Cosine Matrix (DCM).
 */
void quat_to_dcm(Quaternion_t q, float dcm[3][3]) {
    float x = q.x;
    float y = q.y;
    float z = q.z;
    float w = q.w;

    dcm[0][0] = 1.0f - 2.0f * (y*y + z*z);
    dcm[0][1] = 2.0f * (x*y - w*z);
    dcm[0][2] = 2.0f * (x*z + w*y);

    dcm[1][0] = 2.0f * (x*y + w*z);
    dcm[1][1] = 1.0f - 2.0f * (x*x + z*z);
    dcm[1][2] = 2.0f * (y*z - w*x);

    dcm[2][0] = 2.0f * (x*z - w*y);
    dcm[2][1] = 2.0f * (y*z + w*x);
    dcm[2][2] = 1.0f - 2.0f * (x*x + y*y);
}
