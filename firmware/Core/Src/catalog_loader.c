#include "catalog_loader.h"
#include "catalog_metadata.h"
#include "catalog.h"
#include <stddef.h>

bool catalog_init(void) {
    if (CATALOG_NUM_STARS == 0 || CATALOG_NUM_TRIANGLES == 0) {
        return false;
    }
    return true; 
}

uint32_t catalog_get_num_stars(void) {
    return CATALOG_NUM_STARS;
}

uint32_t catalog_get_num_triangles(void) {
    return CATALOG_NUM_TRIANGLES;
}

bool catalog_get_star(uint32_t index, StarVector *out_star) {
    if (index >= CATALOG_NUM_STARS || out_star == NULL) {
        return false;
    }

    out_star->hip_id = (uint32_t)CATALOG_STAR_VECTORS[index][0];
    out_star->x      = CATALOG_STAR_VECTORS[index][1];
    out_star->y      = CATALOG_STAR_VECTORS[index][2];
    out_star->z      = CATALOG_STAR_VECTORS[index][3];

    return true;
}

void catalog_get_star_vector(uint32_t hip_id, Vector3_t *out_vec) {
    for (uint32_t i = 0; i < CATALOG_NUM_STARS; i++) {
        if ((uint32_t)CATALOG_STAR_VECTORS[i][0] == hip_id) {
            out_vec->x = CATALOG_STAR_VECTORS[i][1];
            out_vec->y = CATALOG_STAR_VECTORS[i][2];
            out_vec->z = CATALOG_STAR_VECTORS[i][3];
            return;
        }
    }
    out_vec->x = 0.0f; out_vec->y = 0.0f; out_vec->z = 1.0f;
}

bool catalog_get_triangle(uint32_t index, TriangleEntry *out_triangle) {
    if (index >= CATALOG_NUM_TRIANGLES || out_triangle == NULL) {
        return false;
    }

    out_triangle->angles[0] = CATALOG_TRIANGLES[index][0];
    out_triangle->angles[1] = CATALOG_TRIANGLES[index][1];
    out_triangle->angles[2] = CATALOG_TRIANGLES[index][2];
    
    out_triangle->hips[0]   = (uint32_t)CATALOG_TRIANGLE_IDS[index][0];
    out_triangle->hips[1]   = (uint32_t)CATALOG_TRIANGLE_IDS[index][1];
    out_triangle->hips[2]   = (uint32_t)CATALOG_TRIANGLE_IDS[index][2];

    return true;
}
