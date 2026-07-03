/**
 * @file quest.c
 * @brief Implementation of the QUEST algorithm.
 *
 * This file constructs the B and K matrices from star observations and
 * solves for the optimal attitude quaternion.
 *
 * @author Aditya (WhereSat Team)
 */

#include "quest.h"
#include <string.h>

/**
 * @brief Computes the optimal quaternion using the QUEST algorithm.
 */
Quaternion_t quest_compute(QUEST_Input_t *input) {
    float B[3][3] = {0};
    float S[3][3] = {0};
    float Z[3] = {0};
    float sigma = 0;
    float lambda_max = 0;

    // 1. Build B matrix and Z vector
    for (int i = 0; i < input->count; i++) {
        float w = input->weights[i];
        Vector3_t b = input->body_v[i];
        Vector3_t r = input->reference_v[i];

        // B = sum(w_i * b_i * r_i^T)
        B[0][0] += w * b.x * r.x; B[0][1] += w * b.x * r.y; B[0][2] += w * b.x * r.z;
        B[1][0] += w * b.y * r.x; B[1][1] += w * b.y * r.y; B[1][2] += w * b.y * r.z;
        B[2][0] += w * b.z * r.x; B[2][1] += w * b.z * r.y; B[2][2] += w * b.z * r.z;

        // Z = sum(w_i * (b_i x r_i))
        Z[0] += w * (b.y * r.z - b.z * r.y);
        Z[1] += w * (b.z * r.x - b.x * r.z);
        Z[2] += w * (b.x * r.y - b.y * r.x);

        lambda_max += w; // Initial guess for eigenvalue
    }

    // 2. Compute S = B + B^T and sigma = trace(B)
    sigma = B[0][0] + B[1][1] + B[2][2];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            S[i][j] = B[i][j] + B[j][i];
        }
    }

    // 3. Solve for Quaternion (Simplified Davenport solution)
    // We solve: [(lambda_max + sigma)I - S] * Y = Z
    // Then q = [1, Y] normalized.

    float alpha = lambda_max + sigma;
    float M[3][3];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            M[i][j] = (i == j) ? (alpha - S[i][j]) : (-S[i][j]);
        }
    }

    // Simple Cramer's rule or Gaussian elimination to solve M*Y = Z
    // For brevity in this port, we use a basic 3x3 inversion
    float det = M[0][0]*(M[1][1]*M[2][2] - M[1][2]*M[2][1]) -
                M[0][1]*(M[1][0]*M[2][2] - M[1][2]*M[2][0]) +
                M[0][2]*(M[1][0]*M[2][1] - M[1][1]*M[2][0]);

    Quaternion_t q_out = {1, 0, 0, 0};
    if (fabsf(det) > 1e-6f) {
        q_out.q1 = ((M[1][1]*M[2][2] - M[1][2]*M[2][1])*Z[0] + (M[0][2]*M[2][1] - M[0][1]*M[2][2])*Z[1] + (M[0][1]*M[1][2] - M[0][2]*M[1][1])*Z[2]) / det;
        q_out.q2 = ((M[1][2]*M[2][0] - M[1][0]*M[2][2])*Z[0] + (M[0][0]*M[2][2] - M[0][2]*M[2][0])*Z[1] + (M[0][2]*M[1][0] - M[0][0]*M[1][2])*Z[2]) / det;
        q_out.q3 = ((M[1][0]*M[2][1] - M[1][1]*M[2][0])*Z[0] + (M[0][1]*M[2][0] - M[0][0]*M[2][1])*Z[1] + (M[0][0]*M[1][1] - M[0][1]*M[1][0])*Z[2]) / det;
        q_out.q0 = 1.0f;
    }

    return quat_normalize(q_out);
}
