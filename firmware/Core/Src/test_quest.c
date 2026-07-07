#include "test_pipeline.h"
#include "quest.h"
#include "catalog_loader.h"
#include <stdio.h>
#include <math.h>

bool test_quest_module(void) {
    printf("  [TEST] Running QUEST Module...\r\n");

    // 1. ARRANGE: The Golden Reference Expected Quaternion [w, x, y, z]
    float expected_q[4] = {0.215794f, -0.951054f, -0.016173f, 0.220605f}; 
    
    // Setup the fake input for QUEST
    QUEST_Input_t quest_data;
    quest_data.count = 5;
    uint32_t test_hips[5] = {66447, 68483, 71453, 71456, 71634};
    
    // Fake Body Vectors (from Python)
    quest_data.body_v[0] = (Vector3_t){-0.127866f, -0.164279f, 0.978091f};
    quest_data.body_v[1] = (Vector3_t){0.004759f, -0.173535f, 0.984816f};
    quest_data.body_v[2] = (Vector3_t){0.055455f, -0.115974f, 0.991703f};
    quest_data.body_v[3] = (Vector3_t){0.055430f, -0.115886f, 0.991715f};
    quest_data.body_v[4] = (Vector3_t){-0.122277f, 0.036667f, 0.991819f};

    for(int i=0; i<5; i++) {
        catalog_get_star_vector(test_hips[i], &quest_data.reference_v[i]);
        quest_data.weights[i] = 1.0f;
    }

    // 2. ACT: Run QUEST
    Quaternion_t actual_q = quest_compute(&quest_data);

    // 3. ASSERT: Grade the floats using an Epsilon of 0.001
    float epsilon = 0.001f;
    if (fabsf(actual_q.q0 - expected_q[0]) > epsilon ||
        fabsf(actual_q.q1 - expected_q[1]) > epsilon ||
        fabsf(actual_q.q2 - expected_q[2]) > epsilon ||
        fabsf(actual_q.q3 - expected_q[3]) > epsilon) {
        
        printf("    -> [FAIL] Quaternion mismatch!\r\n");
        printf("       Expected: [%.6f, %.6f, %.6f, %.6f]\r\n", expected_q[0], expected_q[1], expected_q[2], expected_q[3]);
        printf("       Got:      [%.6f, %.6f, %.6f, %.6f]\r\n", actual_q.q0, actual_q.q1, actual_q.q2, actual_q.q3);
        return false;
    }

    printf("    -> [PASS] QUEST Quaternion matched Golden Reference.\r\n");
    return true;
}