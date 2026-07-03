#ifndef CATALOG_LOADER_H
#define CATALOG_LOADER_H

#include <stdint.h>
#include <stdbool.h>
#include "camera_geometry.h"
// ---------------------------------------------------------
// Data Structures
// ---------------------------------------------------------

// Holds the 3D unit vector for a specific star
typedef struct {
    uint32_t hip_id;
    float x;
    float y;
    float z;
} StarVector;

// Holds the fingerprint (3 angles) and the 3 star IDs for a triangle
typedef struct {
    float angles[3];   // Smallest, Middle, Largest angle (in radians)
    uint32_t hips[3];  // The 3 Hipparcos IDs corresponding to this triangle
} TriangleEntry;

// ---------------------------------------------------------
// Function Prototypes
// ---------------------------------------------------------

// Initializes and verifies the catalog
bool catalog_init(void);

// Returns the total number of items in the database
uint32_t catalog_get_num_stars(void);
uint32_t catalog_get_num_triangles(void);
void catalog_get_star_vector(uint32_t hip_id, Vector3_t *out_vec);
// Fetches data at a specific index. Returns false if index is out of bounds.
bool catalog_get_star(uint32_t index, StarVector *out_star);
bool catalog_get_triangle(uint32_t index, TriangleEntry *out_triangle);

#endif // CATALOG_LOADER_H
