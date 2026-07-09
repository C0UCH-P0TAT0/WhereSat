#include "test_pipeline.h"
#include "quest.h"
#include "catalog_loader.h"
#include <stdio.h>
#include <math.h>

bool test_quest_module(void) {
    printf("  [TEST] Running QUEST Module...\r\n");

    // 1. ARRANGE: The Golden Reference Expected Quaternion [w, x, y, z]
    float expected_q[4] = {0.537193f, 0.599655f, -0.449512f, -0.387009f}; 
    
    // Setup the fake input for QUEST
    QUEST_Input_t quest_data;
    quest_data.count = 5;
    uint32_t test_hips[5] = {24378, 25865, 26246, 26662, 27298};
    
    // Fake Body Vectors (from Python)
    quest_data.body_v[0] = (Vector3_t){0.073255f, -0.162152f, 0.984043f};
    quest_data.body_v[1] = (Vector3_t){-0.081340f, -0.133382f, 0.987721f};
    quest_data.body_v[2] = (Vector3_t){-0.072059f, -0.111312f, 0.991170f};
    quest_data.body_v[3] = (Vector3_t){-0.065864f, -0.088633f, 0.993884f};
    quest_data.body_v[4] = (Vector3_t){0.053295f, -0.018924f, 0.998399f};

    for(int i=0; i<5; i++) {
        catalog_get_star_vector(test_hips[i], &quest_data.reference_v[i]);
        quest_data.weights[i] = 1.0f;
    }

    // 2. ACT: Run QUEST
    Quaternion_t actual_q = quest_compute(&quest_data);

    // 3. ASSERT: Grade the floats using an Epsilon of 0.001
    float epsilon = 0.001f;
    if (fabsf(actual_q.w - expected_q[0]) > epsilon ||
        fabsf(actual_q.x - expected_q[1]) > epsilon ||
        fabsf(actual_q.y - expected_q[2]) > epsilon ||
        fabsf(actual_q.z - expected_q[3]) > epsilon) {
        
        printf("    -> [FAIL] Quaternion mismatch!\r\n");
        printf("       Expected: [%.6f, %.6f, %.6f, %.6f]\r\n", expected_q[0], expected_q[1], expected_q[2], expected_q[3]);
        printf("       Got:      [%.6f, %.6f, %.6f, %.6f]\r\n", actual_q.w, actual_q.x, actual_q.y, actual_q.z);
        return false;
    }

    printf("    -> [PASS] QUEST Quaternion matched Golden Reference.\r\n");
    return true;
}