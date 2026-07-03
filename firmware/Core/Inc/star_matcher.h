#ifndef STAR_MATCHER_H
#define STAR_MATCHER_H

#include <stdint.h>
#include <stdbool.h>
#include "triangle_builder.h"
#include "catalog_search.h"

// ---------------------------------------------------------
// Tuning Parameters
// ---------------------------------------------------------
// A star must receive at least this many votes to be considered a "True Match".
// Since 1 star is usually part of many triangles, 3 is a safe minimum.
#define MIN_VOTES_REQUIRED 3

// Maximum number of unique HIP IDs we will track votes for per observed star
#define MAX_TRACKED_HIPS 5 

// ---------------------------------------------------------
// Data Structures
// ---------------------------------------------------------
// Holds the final result: mapping a camera star to a real database star
typedef struct {
    uint8_t local_id;    // The ID from the camera (0 to 9)
    uint32_t hip_id;     // The matched Hipparcos ID
    bool is_matched;     // True if we found a confident match, False if not
    uint8_t vote_count;  // How many votes this match got (for debugging/confidence)
} MatchedStar;

// ---------------------------------------------------------
// Function Prototypes
// ---------------------------------------------------------

// The main function to cross-reference all triangles and find the true stars
void match_stars(const ObservedTriangle *triangles, uint16_t num_triangles, 
                 uint8_t num_observed_stars, MatchedStar *out_matches);

#endif // STAR_MATCHER_H
