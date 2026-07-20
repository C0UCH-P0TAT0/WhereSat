/**
 * @file triangle_builder.c
 * @brief Implementation of the Star Triangle Generation logic with Debug Tracing.
 * 
 * This module takes identified star vectors and generates all possible 
 * combinations of 3-star triangles, sorting their internal angles to create 
 * a unique "fingerprint" for catalog matching.
 * 
 * @author Aditya (WhereSat Team)
 */

#include "triangle_builder.h"
#include <math.h>
#include <stdio.h>

// ---------------------------------------------------------
// Helper: Sort 3 Angles
// ---------------------------------------------------------
void sort_triangle_angles(float *angles) {
    float temp;
    if (angles[0] > angles[1]) {
        temp = angles[0]; angles[0] = angles[1]; angles[1] = temp;
    }
    if (angles[1] > angles[2]) {
        temp = angles[1]; angles[1] = angles[2]; angles[2] = temp;
    }
    if (angles[0] > angles[1]) {
        temp = angles[0]; angles[0] = angles[1]; angles[1] = temp;
    }
}

// ---------------------------------------------------------
// Helper: Calculate Angle Between Two Vectors
// ---------------------------------------------------------
static float calculate_angle(const ObservedStar *s1, const ObservedStar *s2) {
    float dot = (s1->x * s2->x) + (s1->y * s2->y) + (s1->z * s2->z);
    
    // Clamp to prevent NaN from precision errors
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    
    return acosf(dot);
}

void build_triangles(const ObservedStar *stars, uint8_t num_stars, 
                     ObservedTriangle *out_triangles, uint16_t *out_num_triangles) {
    
    uint16_t triangle_count = 0;

    printf("\r\n[Builder] Starting Triangle Generation (Stars: %d)\r\n", num_stars);

    if (num_stars < 3) {
        printf("  [Builder] ERROR: Not enough stars to form a triangle.\r\n");
        *out_num_triangles = 0;
        return;
    }

    if (num_stars > MAX_OBSERVED_STARS) {
        num_stars = MAX_OBSERVED_STARS;
    }

    // 3 Nested Loops for combinations
    for (uint8_t i = 0; i < num_stars - 2; i++) {
        for (uint8_t j = i + 1; j < num_stars - 1; j++) {
            for (uint8_t k = j + 1; k < num_stars; k++) {
                
                if (triangle_count >= MAX_OBSERVED_TRIANGLES) {
                    break;
                }

                // Calculate angles
                float d_ij = calculate_angle(&stars[i], &stars[j]);
                float d_jk = calculate_angle(&stars[j], &stars[k]);
                float d_ik = calculate_angle(&stars[i], &stars[k]);

                // Store data
                out_triangles[triangle_count].angles[0] = d_ij;
                out_triangles[triangle_count].angles[1] = d_jk;
                out_triangles[triangle_count].angles[2] = d_ik;

                out_triangles[triangle_count].star_indices[0] = stars[i].local_id;
                out_triangles[triangle_count].star_indices[1] = stars[j].local_id;
                out_triangles[triangle_count].star_indices[2] = stars[k].local_id;

                // Sort angles for the fingerprint
                sort_triangle_angles(out_triangles[triangle_count].angles);

                // --- DEBUG TRACE: Print first 10 triangles ---
            

                triangle_count++;
            }
        }
    }

    *out_num_triangles = triangle_count;
    printf("[Builder] Finished. Total Triangles Built: %d\r\n", triangle_count);
}