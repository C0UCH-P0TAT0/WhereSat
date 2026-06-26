#include "catalog_search.h"
#include <math.h>

// ---------------------------------------------------------
// Helper: Binary Search for the Smallest Angle
// ---------------------------------------------------------
// Returns the index of the triangle in the database whose smallest angle
// is closest to our target_angle.
uint32_t binary_search_shortest_angle(float target_angle) {
    int32_t low = 0;
    int32_t high = catalog_get_num_triangles() - 1;
    uint32_t closest_index = 0;
    float min_diff = 999.0f; // Start with an impossibly large difference

    TriangleEntry temp_tri;

    while (low <= high) {
        int32_t mid = low + (high - low) / 2;
        
        // Fetch the triangle at the middle index
        if (!catalog_get_triangle(mid, &temp_tri)) {
            break; // Safety catch
        }

        float mid_angle = temp_tri.angles[0];
        float diff = fabsf(mid_angle - target_angle);

        // Keep track of the absolute closest match we've seen
        if (diff < min_diff) {
            min_diff = diff;
            closest_index = mid;
        }

        // Standard binary search logic
        if (mid_angle < target_angle) {
            low = mid + 1;
        } else if (mid_angle > target_angle) {
            high = mid - 1;
        } else {
            // Exact match found (rare with floats, but possible)
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

    // 2. Scan Backwards (Down the array)
    // Because of the tolerance window, earlier indices might also be valid!
    int32_t scan_idx = start_idx;
    TriangleEntry cat_tri;

    while (scan_idx >= 0) {
        catalog_get_triangle(scan_idx, &cat_tri);
        
        // If the smallest angle falls outside our tolerance window, stop scanning backwards
        if (fabsf(cat_tri.angles[0] - target_small_angle) > MATCH_TOLERANCE) {
            break;
        }

        // Check if the Middle and Largest angles ALSO match within tolerance
        if (fabsf(cat_tri.angles[1] - observed->angles[1]) <= MATCH_TOLERANCE &&
            fabsf(cat_tri.angles[2] - observed->angles[2]) <= MATCH_TOLERANCE) {
            
            // We found a candidate! Save it.
            if (*out_num_candidates < MAX_CANDIDATES) {
                out_candidates[*out_num_candidates].catalog_index = scan_idx;
                out_candidates[*out_num_candidates].hips[0] = cat_tri.hips[0];
                out_candidates[*out_num_candidates].hips[1] = cat_tri.hips[1];
                out_candidates[*out_num_candidates].hips[2] = cat_tri.hips[2];
                (*out_num_candidates)++;
            }
        }
        scan_idx--;
    }

    // 3. Scan Forwards (Up the array)
    // We start at start_idx + 1 because we already checked start_idx in the backwards scan
    scan_idx = start_idx + 1;
    while (scan_idx < total_triangles) {
        catalog_get_triangle(scan_idx, &cat_tri);
        
        // If the smallest angle falls outside our tolerance window, stop scanning forwards
        if (fabsf(cat_tri.angles[0] - target_small_angle) > MATCH_TOLERANCE) {
            break;
        }

        // Check Middle and Largest angles
        if (fabsf(cat_tri.angles[1] - observed->angles[1]) <= MATCH_TOLERANCE &&
            fabsf(cat_tri.angles[2] - observed->angles[2]) <= MATCH_TOLERANCE) {
            
            // We found a candidate! Save it.
            if (*out_num_candidates < MAX_CANDIDATES) {
                out_candidates[*out_num_candidates].catalog_index = scan_idx;
                out_candidates[*out_num_candidates].hips[0] = cat_tri.hips[0];
                out_candidates[*out_num_candidates].hips[1] = cat_tri.hips[1];
                out_candidates[*out_num_candidates].hips[2] = cat_tri.hips[2];
                (*out_num_candidates)++;
            }
        }
        scan_idx++;
    }
}