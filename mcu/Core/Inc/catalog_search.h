#ifndef CATALOG_SEARCH_H
#define CATALOG_SEARCH_H

#include <stdint.h>
#include "triangle_builder.h"
#include "catalog_loader.h"


// The maximum allowed error between the camera's angle and the database's angle.
#define MATCH_TOLERANCE 0.004f

// Maximum number of candidate matches we can store for a single triangle
#define MAX_CANDIDATES 10


typedef struct {
    uint32_t catalog_index; // Where we found it in the database
    uint32_t hips[3];       // The 3 Star IDs from the database
} CandidateMatch;


// Performs a binary search to find the closest matching smallest angle
uint32_t binary_search_shortest_angle(float target_angle);

// Finds all database triangles that match the observed triangle within tolerance
void find_candidate_triangles(const ObservedTriangle *observed, 
                              CandidateMatch *out_candidates, 
                              uint8_t *out_num_candidates);

#endif // CATALOG_SEARCH_H
