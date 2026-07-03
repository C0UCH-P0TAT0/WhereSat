/**
 * @file quaternion.h
 * @brief Shared quaternion library for QUEST, MEKF, and Control logic.
 *
 * Provides standard 4-element quaternion structures and operations including
 * multiplication, conjugation, and vector rotation.
 *
 * @author Aditya (WhereSat Team)
 */

#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

#include "camera_geometry.h" // For Vector3_t

/**
 * @brief Quaternion structure [q0, q1, q2, q3] where q0 is scalar.
 */
typedef struct {
    float q0; // Scalar part
    float q1; // Vector x
    float q2; // Vector y
    float q3; // Vector z
} Quaternion_t;

/* Function Prototypes */
Quaternion_t quat_normalize(Quaternion_t q);
Quaternion_t quat_multiply(Quaternion_t q, Quaternion_t p);
Quaternion_t quat_conjugate(Quaternion_t q);
Vector3_t quat_rotate_vector(Quaternion_t q, Vector3_t v);
void quat_to_dcm(Quaternion_t q, float dcm[3][3]);

#endif /* INC_QUATERNION_H_ */
