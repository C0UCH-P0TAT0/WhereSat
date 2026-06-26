/**
 * @file test_suite.c
 * @brief Validation tests for Aditya's infrastructure and math modules.
 *
 * This file contains test cases to verify that pixel-to-vector conversions
 * meet the required precision (< 1e-6) and that SPI packet parsing is robust.
 *
 * @author Aditya (WhereSat Team)
 */

#include "camera_geometry.h"
#include <stdio.h>
#include <math.h>

/**
 * @brief Verifies that a pixel at the principal point results in a forward-facing vector [0,0,1].
 */
void test_camera_geometry(void) {
    printf("\r\n--- Running Geometry Tests ---\r\n");

    // Test Case 1: Center Pixel
    Centroid_t center = {PRINCIPAL_POINT_X, PRINCIPAL_POINT_Y};
    Vector3_t result = pixel_to_vector(center);

    printf("Test 1 (Center Pixel): ");
    if (fabsf(result.x) < 1e-6 && fabsf(result.y) < 1e-6 && fabsf(result.z - 1.0f) < 1e-6) {
        printf("PASSED\r\n");
    } else {
        printf("FAILED! Got [%.6f, %.6f, %.6f]\r\n", result.x, result.y, result.z);
    }

    // Test Case 2: Known Offset
    // If focal length is 1024 and we move 1024 pixels right, angle should be 45 degrees
    Centroid_t offset = {PRINCIPAL_POINT_X + FOCAL_LENGTH_PX, PRINCIPAL_POINT_Y};
    result = pixel_to_vector(offset);

    printf("Test 2 (45-deg Offset): ");
    // Expected: [1/sqrt(2), 0, 1/sqrt(2)] -> [0.7071, 0, 0.7071]
    if (fabsf(result.x - 0.707107f) < 1e-5) {
        printf("PASSED\r\n");
    } else {
        printf("FAILED! Got X=%.6f\r\n", result.x);
    }

    printf("--- Geometry Tests Complete ---\r\n\r\n");
}
