#ifndef STAR_MATCHER_H
#define STAR_MATCHER_H

#include <stdint.h>
#include <stdbool.h>
#include "triangle_builder.h"
#include "catalog_search.h"


#define MIN_VOTES_REQUIRED 3
#define MAX_TRACKED_HIPS 5 

// Geometric dot product threshold (0.002 ~ 2.5 degrees of tolerance)
#define GEOMETRIC_TOLERANCE_DOT 0.002f 


typedef struct {
    uint8_t local_id;    
    uint32_t hip_id;     
    bool is_matched;     
    uint8_t vote_count;  
} MatchedStar;


void match_stars(const ObservedStar *observed_stars, const ObservedTriangle *triangles, 
                 uint16_t num_triangles, uint8_t num_observed_stars, 
                 MatchedStar *out_matches);

#endif // STAR_MATCHER_H