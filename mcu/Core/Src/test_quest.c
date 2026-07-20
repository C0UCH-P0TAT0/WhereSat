/**
 * @file test_quest.c
 * @brief Automated regression test for the QUEST module.
 *
 * Verifies that the C implementation of the Davenport q-method matches
 * the Python reference using the Scalar-Last [x, y, z, w] convention.
 *
 * @author Aditya (WhereSat Team)
 */

#include "test_pipeline.h"
#include "quest.h"
#include "catalog_loader.h"
#include <stdio.h>
#include <math.h>

/**
 * @brief Runs a regression test against a known "Golden" star set.
 * @return true if actual quaternion matches expected within epsilon.
 */
bool test_quest_module(void) {
    printf("  [TEST] Running QUEST Module...\r\n");

    // 1. ARRANGE: The Golden Reference Expected Quaternion [x, y, z, w]
    
    float expected_q[4] = {
        -0.951054f, // x
        -0.016173f, // y
         0.220605f, // z
         0.215794f  // w (scalar)
    };
    
    // Setup the input for QUEST
    QUEST_Input_t quest_data;
    quest_data.count = 5;
    uint32_t test_hips[5] = {66447, 68483, 71453, 71456, 71634};
    
    // Body Vectors (from Python simulation)
    quest_data.body_v[0] = (Vector3_t){-0.127866f, -0.164279f, 0.978091f};
    quest_data.body_v[1] = (Vector3_t){ 0.004759f, -0.173535f, 0.984816f};
    quest_data.body_v[2] = (Vector3_t){ 0.055455f, -0.115974f, 0.991703f};
    quest_data.body_v[3] = (Vector3_t){ 0.055430f, -0.115886f, 0.991715f};
    quest_data.body_v[4] = (Vector3_t){-0.122277f,  0.036667f, 0.991819f};

    for(int i = 0; i < 5; i++) {
        catalog_get_star_vector(test_hips[i], &quest_data.reference_v[i]);
        quest_data.weights[i] = 1.0f;
    }

    // 2. ACT: Run QUEST
    Quaternion_t actual_q = quest_compute(&quest_data);

    // 3. ASSERT: Grade the floats using an Epsilon of 0.001
    float epsilon = 0.001f;

    // Compare using new Scalar-Last members: x, y, z, w
    if (fabsf(actual_q.x - expected_q[0]) > epsilon ||
        fabsf(actual_q.y - expected_q[1]) > epsilon ||
        fabsf(actual_q.z - expected_q[2]) > epsilon ||
        fabsf(actual_q.w - expected_q[3]) > epsilon) {
        
        printf("    -> [FAIL] Quaternion mismatch!\r\n");
        printf("       Expected [x,y,z,w]: [%.6f, %.6f, %.6f, %.6f]\r\n",
                expected_q[0], expected_q[1], expected_q[2], expected_q[3]);
        printf("       Got      [x,y,z,w]: [%.6f, %.6f, %.6f, %.6f]\r\n",
                actual_q.x, actual_q.y, actual_q.z, actual_q.w);
        return false;
    }

    printf("    -> [PASS] QUEST Quaternion matched Golden Reference.\r\n");
    return true;
}
