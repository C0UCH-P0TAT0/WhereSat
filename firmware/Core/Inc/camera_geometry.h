/**
 * @file camera_geometry.h
 * @brief Math utilities for camera-to-body frame transformations.
 *
 * Defines the camera intrinsic parameters (focal length, principal point)
 * and functions to project pixel coordinates into 3D space.
 *
 * @author Aditya (WhereSat Team)
 */

#ifndef INC_CAMERA_GEOMETRY_H_
#define INC_CAMERA_GEOMETRY_H_

#include "fpga_interface.h"

/**
 * @brief 3D Vector structure for star unit vectors.
 */
typedef struct {
    float x;
    float y;
    float z;
} Vector3_t;

/* Camera Intrinsics (Matched to Python Golden Reference) */
#define FOCAL_LENGTH_PX 2903.71f // 20-degree FOV
#define PRINCIPAL_POINT_X 512.0f // Center of 1024x1024 sensor
#define PRINCIPAL_POINT_Y 512.0f // Center of 1024x1024 sensor

/* Function Prototypes */
Vector3_t pixel_to_vector(Centroid_t centroid);
float vector_norm(Vector3_t v);
Vector3_t normalize_vector(Vector3_t v);

#endif /* INC_CAMERA_GEOMETRY_H_ */
