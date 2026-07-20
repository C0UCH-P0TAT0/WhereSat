#ifndef TRIANGLE_BUILDER_H
#define TRIANGLE_BUILDER_H

#include <stdint.h>


// The FPGA passes the top 10 brightest stars. 
#define MAX_OBSERVED_STARS 12

// 12 stars can form a maximum of 220 unique triangles (12 Choose 3)
#define MAX_OBSERVED_TRIANGLES 260 



// Represents a single star seen by the camera (provided by Aditya's code)
typedef struct {
    uint8_t local_id; // The index of the star from the FPGA (0 to 9)
    float x;
    float y;
    float z;
} ObservedStar;

// Represents a triangle built from 3 observed stars
typedef struct {
    float angles[3];         // Sorted angles: Smallest, Middle, Largest (radians)
    uint8_t star_indices[3]; // The local_ids of the 3 stars that make this triangle
} ObservedTriangle;



// Takes an array of observed stars and generates all possible sorted triangles
void build_triangles(const ObservedStar *stars, uint8_t num_stars, 
                     ObservedTriangle *out_triangles, uint16_t *out_num_triangles);

// Helper function to sort 3 angles from smallest to largest
void sort_triangle_angles(float *angles);

#endif // TRIANGLE_BUILDER_H