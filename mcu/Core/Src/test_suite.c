/**
 * @file test_suite.c
 * @brief Comprehensive math validation for camera geometry.
 *
 * Verifies pixel-to-vector projection across all quadrants and ensures
 * all output vectors are unit-length (magnitude = 1.0).
 *
 * @author Aditya (WhereSat Team)
 */

#include "test_suite.h"
#include "camera_geometry.h"
#include <stdio.h>
#include <math.h>

void test_camera_geometry(void) {
    printf("\r\n--- Running Comprehensive Geometry Tests ---\r\n");

    // Test 1: Center Pixel (Boresight)
    Centroid_t center = {PRINCIPAL_POINT_X, PRINCIPAL_POINT_Y};
    Vector3_t v1 = pixel_to_vector(center);
    printf("Test 1 (Center): ");
    if (fabsf(v1.x) < 1e-6 && fabsf(v1.y) < 1e-6 && fabsf(v1.z - 1.0f) < 1e-6) printf("PASSED\r\n");
    else printf("FAILED! [%.4f, %.4f, %.4f]\r\n", v1.x, v1.y, v1.z);

    // Test 2: 45-deg Right (Verify X, Y, and Z)
    Centroid_t right = {PRINCIPAL_POINT_X + FOCAL_LENGTH_PX, PRINCIPAL_POINT_Y};
    Vector3_t v2 = pixel_to_vector(right);
    printf("Test 2 (45-deg Right): ");
    if (fabsf(v2.x - 0.707107f) < 1e-5 && fabsf(v2.y) < 1e-6 && fabsf(v2.z - 0.707107f) < 1e-5) printf("PASSED\r\n");
    else printf("FAILED! [%.4f, %.4f, %.4f]\r\n", v2.x, v2.y, v2.z);

    // Test 3: 45-deg Left (-45°)
    Centroid_t left = {PRINCIPAL_POINT_X - FOCAL_LENGTH_PX, PRINCIPAL_POINT_Y};
    Vector3_t v3 = pixel_to_vector(left);
    printf("Test 3 (45-deg Left): ");
    if (fabsf(v3.x + 0.707107f) < 1e-5 && fabsf(v3.y) < 1e-6 && fabsf(v3.z - 0.707107f) < 1e-5) printf("PASSED\r\n");
    else printf("FAILED! [%.4f, %.4f, %.4f]\r\n", v3.x, v3.y, v3.z);

    // Test 4: 45-deg Up (-45° in Y-axis)
    Centroid_t up = {PRINCIPAL_POINT_X, PRINCIPAL_POINT_Y - FOCAL_LENGTH_PX};
    Vector3_t v4 = pixel_to_vector(up);
    printf("Test 4 (45-deg Up): ");
    if (fabsf(v4.x) < 1e-6 && fabsf(v4.y + 0.707107f) < 1e-5 && fabsf(v4.z - 0.707107f) < 1e-5) printf("PASSED\r\n");
    else printf("FAILED! [%.4f, %.4f, %.4f]\r\n", v4.x, v4.y, v4.z);

    // Test 5: Normalization (Verify Magnitude == 1.0)
    float mag = vector_norm(v4);
    printf("Test 5 (Normalization): ");
    if (fabsf(mag - 1.0f) < 1e-6) printf("PASSED\r\n");
    else printf("FAILED! Magnitude: %.6f\r\n", mag);

    printf("--- All Geometry Tests Complete ---\r\n\r\n");
}
