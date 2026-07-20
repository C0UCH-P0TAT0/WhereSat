#include "test_pipeline.h"
#include "camera_geometry.h"
#include "triangle_builder.h"
#include "star_matcher.h"
#include <stdio.h>

bool test_star_id_module(void) {
    printf("  [TEST] Running Star ID Module...\r\n");

    // 1. ARRANGE: The Golden Reference Inputs (Pixels)
    Centroid_t test_pixels[5] = {
        {132.400f, 24.300f},
        {526.032f, 0.337f},
        {674.371f, 172.428f},
        {674.296f, 172.691f},
        {154.017f, 619.347f}
    };
    
    // The Golden Reference Expected Answers (HIP IDs)
    uint32_t expected_hips[5] = {66447, 68483, 71453, 71456, 71634};

    // 2. ACT: Run your pipeline
    ObservedStar live_stars[5];
    for (int i = 0; i < 5; i++) {
        Vector3_t vec = pixel_to_vector(test_pixels[i]);
        live_stars[i].local_id = i;
        live_stars[i].x = vec.x; 
        live_stars[i].y = vec.y; 
        live_stars[i].z = vec.z;
    }

    ObservedTriangle triangles[20];
    uint16_t num_triangles = 0;
    
    // Build the triangles from the 5 live stars
    build_triangles(live_stars, 5, triangles, &num_triangles);

    MatchedStar final_matches[5];
    
    // FIX: Pass all 5 arguments matching star_matcher.h exactly
    match_stars(live_stars, triangles, num_triangles, 5, final_matches);

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