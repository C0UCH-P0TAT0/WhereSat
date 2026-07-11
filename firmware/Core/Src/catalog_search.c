/**
 * @file catalog_search.c
 * @brief Triangle Database Search Engine.
 * 
 * This file performs the binary search and tolerance-window scanning 
 * to find catalog stars that match observed triangles.
 * 
 * @author Aditya (WhereSat Team)
 */

#include "catalog_search.h"
#include <math.h>
#include <stdio.h>

// ---------------------------------------------------------
// Helper: Binary Search for the Smallest Angle
// ---------------------------------------------------------
uint32_t binary_search_shortest_angle(float target_angle) {
    int32_t low = 0;
    int32_t high = catalog_get_num_triangles() - 1;
    uint32_t closest_index = 0;
    float min_diff = 999.0f;

    TriangleEntry temp_tri;

    while (low <= high) {
        int32_t mid = low + (high - low) / 2;
        
        if (!catalog_get_triangle(mid, &temp_tri)) {
            break;
        }

        float mid_angle = temp_tri.angles[0];
        float diff = fabsf(mid_angle - target_angle);

        if (diff < min_diff) {
            min_diff = diff;
            closest_index = mid;
        }

        if (mid_angle < target_angle) {
            low = mid + 1;
        } else if (mid_angle > target_angle) {
            high = mid - 1;
        } else {
            return mid;
        }
    }

    return closest_index;
}

// ---------------------------------------------------------
// Main Function: Find Candidates with Tolerance Window
// ---------------------------------------------------------
void find_candidate_triangles(const ObservedTriangle *observed, 
                              CandidateMatch *out_candidates, 
                              uint8_t *out_num_candidates) {
    
    *out_num_candidates = 0;
    uint32_t total_triangles = catalog_get_num_triangles();
    
    if (total_triangles == 0) return;

    // 1. Find the closest starting point using Binary Search
    float target_small_angle = observed->angles[0];
    uint32_t start_idx = binary_search_shortest_angle(target_small_angle);

    TriangleEntry cat_tri;

    // 2. Scan Backwards
    int32_t scan_idx = start_idx;
    while (scan_idx >= 0) {
        catalog_get_triangle(scan_idx, &cat_tri);
        
        if (fabsf(cat_tri.angles[0] - target_small_angle) > MATCH_TOLERANCE) {
            break;
        }

        if (fabsf(cat_tri.angles[1] - observed->angles[1]) <= MATCH_TOLERANCE &&
            fabsf(cat_tri.angles[2] - observed->angles[2]) <= MATCH_TOLERANCE) {
            
            if (*out_num_candidates < MAX_CANDIDATES) {
                out_candidates[*out_num_candidates].catalog_index = scan_idx;
                out_candidates[*out_num_candidates].hips[0] = cat_tri.hips[0];
                out_candidates[*out_num_candidates].hips[1] = cat_tri.hips[1];
                out_candidates[*out_num_candidates].hips[2] = cat_tri.hips[2];
                (*out_num_candidates)++;
            } else {
                break;
            }
        }
        scan_idx--;
    }

    // 3. Scan Forwards
    scan_idx = start_idx + 1;
    while (scan_idx < total_triangles) {
        catalog_get_triangle(scan_idx, &cat_tri);
        
        if (fabsf(cat_tri.angles[0] - target_small_angle) > MATCH_TOLERANCE) {
            break;
        }

        if (fabsf(cat_tri.angles[1] - observed->angles[1]) <= MATCH_TOLERANCE &&
            fabsf(cat_tri.angles[2] - observed->angles[2]) <= MATCH_TOLERANCE) {
            
            if (*out_num_candidates < MAX_CANDIDATES) {
                out_candidates[*out_num_candidates].catalog_index = scan_idx;
                out_candidates[*out_num_candidates].hips[0] = cat_tri.hips[0];
                out_candidates[*out_num_candidates].hips[1] = cat_tri.hips[1];
                out_candidates[*out_num_candidates].hips[2] = cat_tri.hips[2];
                (*out_num_candidates)++;
            } else {
                break;
            }
        }
        scan_idx++;
    }
}