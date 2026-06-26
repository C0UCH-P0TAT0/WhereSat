/**
 * @file camera_geometry.c
 * @brief Implementation of pixel-to-unit-vector conversion.
 *
 * Converts 2D sensor coordinates into normalized 3D vectors using
 * the pinhole camera model. These vectors are used for star matching.
 *
 * @author Aditya (WhereSat Team)
 */

#include "camera_geometry.h"
#include <math.h>

/**
 * @brief Converts a pixel (X, Y) to a 3D vector in the camera frame.
 * Formula: V = [(x - cx), (y - cy), f]
 */
Vector3_t pixel_to_vector(Centroid_t centroid) {
    Vector3_t v;
    v.x = (centroid.x - PRINCIPAL_POINT_X);
    v.y = (centroid.y - PRINCIPAL_POINT_Y);
    v.z = FOCAL_LENGTH_PX;

    return normalize_vector(v);
}

/**
 * @brief Calculates the L2 norm (magnitude) of a 3D vector.
 */
float vector_norm(Vector3_t v) {
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

/**
 * @brief Normalizes a vector to unit length (magnitude = 1.0).
 */
Vector3_t normalize_vector(Vector3_t v) {
    float mag = vector_norm(v);
    Vector3_t unit_v = {0, 0, 0};

    if (mag > 0.00001f) {
        unit_v.x = v.x / mag;
        unit_v.y = v.y / mag;
        unit_v.z = v.z / mag;
    }

    return unit_v;
}
