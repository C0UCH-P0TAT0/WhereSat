#include "triangle_builder.h"
#include <math.h>

// ---------------------------------------------------------
// Helper: Sort 3 Angles
// ---------------------------------------------------------
// A highly optimized, hardcoded sort for exactly 3 elements.
// We don't need a heavy algorithm like QuickSort for just 3 numbers.
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
    // 1. Calculate Dot Product
    float dot = (s1->x * s2->x) + (s1->y * s2->y) + (s1->z * s2->z);
    
    // 2. Clamp the dot product to [-1.0, 1.0]
    // Floating point math can sometimes produce 1.0000001, which causes acosf() 
    // to return NaN (Not a Number) and crash the math engine.
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;
    
    // 3. Return the angle in radians using the hardware FPU
    return acosf(dot);
}

// ---------------------------------------------------------
// Main Function: Build All Triangles
// ---------------------------------------------------------
void build_triangles(const ObservedStar *stars, uint8_t num_stars, 
                     ObservedTriangle *out_triangles, uint16_t *out_num_triangles) {
    
    uint16_t triangle_count = 0;

    // Safety check: We need at least 3 stars to make a triangle
    if (num_stars < 3) {
        *out_num_triangles = 0;
        return;
    }

    // Cap the number of stars to our static memory limit
    if (num_stars > MAX_OBSERVED_STARS) {
        num_stars = MAX_OBSERVED_STARS;
    }

    // 3 Nested Loops to get every unique combination of 3 stars
    for (uint8_t i = 0; i < num_stars - 2; i++) {
        for (uint8_t j = i + 1; j < num_stars - 1; j++) {
            for (uint8_t k = j + 1; k < num_stars; k++) {
                
                // Prevent buffer overflow
                if (triangle_count >= MAX_OBSERVED_TRIANGLES) {
                    break;
                }

                // Calculate the 3 internal angles of this triangle
                float d_ij = calculate_angle(&stars[i], &stars[j]);
                float d_jk = calculate_angle(&stars[j], &stars[k]);
                float d_ik = calculate_angle(&stars[i], &stars[k]);

                // Store the angles in our output struct
                out_triangles[triangle_count].angles[0] = d_ij;
                out_triangles[triangle_count].angles[1] = d_jk;
                out_triangles[triangle_count].angles[2] = d_ik;

                // Store which stars made this triangle
                out_triangles[triangle_count].star_indices[0] = stars[i].local_id;
                out_triangles[triangle_count].star_indices[1] = stars[j].local_id;
                out_triangles[triangle_count].star_indices[2] = stars[k].local_id;

                // Sort the angles (Smallest, Middle, Largest)
                sort_triangle_angles(out_triangles[triangle_count].angles);

                triangle_count++;
            }
        }
    }

    // Return the total number of triangles we successfully built
    *out_num_triangles = triangle_count;
}