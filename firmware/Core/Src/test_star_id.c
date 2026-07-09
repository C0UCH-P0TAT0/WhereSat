#include "test_pipeline.h"
#include "camera_geometry.h"
#include "triangle_builder.h"
#include "star_matcher.h"
#include <stdio.h>

bool test_star_id_module(void) {
    printf("  [TEST] Running Star ID Module...\r\n");

    // 1. ARRANGE: The Golden Reference Inputs (Pixels)
    Centroid_t test_pixels[5] = {
        {728.159f, 33.526f},
        {272.877f, 119.884f},
        {300.900f, 185.904f},
        {319.573f, 253.054f},
        {667.001f, 456.964f}
    };
    
    // The Golden Reference Expected Answers (HIP IDs)
    uint32_t expected_hips[5] = {24378, 25865, 26246, 26662, 27298};

    // 2. ACT: Run your pipeline
    ObservedStar live_stars[5];
    for (int i = 0; i < 5; i++) {
        Vector3_t vec = pixel_to_vector(test_pixels[i]);
        live_stars[i].local_id = i;
        live_stars[i].x = vec.x; live_stars[i].y = vec.y; live_stars[i].z = vec.z;
    }

    ObservedTriangle triangles[20];
    uint16_t num_triangles = 0;
    build_triangles(live_stars, 5, triangles, &num_triangles);

    MatchedStar final_matches[5];
    match_stars(triangles, num_triangles, 5, final_matches);

    // 3. ASSERT: Grade the answers
    for (int i = 0; i < 5; i++) {
        if (!final_matches[i].is_matched || final_matches[i].hip_id != expected_hips[i]) {
            printf("    -> [FAIL] Star %d: Expected %lu, Got %lu\r\n", 
                   i, expected_hips[i], final_matches[i].hip_id);
            return false; // Test failed!
        }
    }

    printf("    -> [PASS] All 5 HIP IDs matched perfectly.\r\n");
    return true;
}