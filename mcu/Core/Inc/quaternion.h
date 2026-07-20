/**
 * @file quaternion.h
 * @brief Quaternion math library using Scalar-Last [x, y, z, w] convention.
 *
 * This header defines the 4-element quaternion structure where the
 * scalar component (w) is stored at the end of the struct.
 *
 * @author Aditya (WhereSat Team)
 */

#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

#include "camera_geometry.h"

/**
 * @brief Quaternion structure [x, y, z, w] where w is scalar.
 */
typedef struct {
    float x;
    float y;
    float z;
    float w; // Scalar component
} Quaternion_t;

/* Function Prototypes */
Quaternion_t quat_normalize(Quaternion_t q);
Quaternion_t quat_multiply(Quaternion_t q, Quaternion_t p);
Quaternion_t quat_conjugate(Quaternion_t q);
Vector3_t quat_rotate_vector(Quaternion_t q, Vector3_t v);
void quat_to_dcm(Quaternion_t q, float dcm[3][3]);

#endif /* INC_QUATERNION_H_ */
