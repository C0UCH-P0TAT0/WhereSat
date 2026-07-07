/**
 * @file quest.c
 * @brief QUEST / Davenport q-method implementation (Scalar-Last [x, y, z, w]).
 * 
 * This implementation constructs the Davenport K-matrix in the vector-first
 * layout to match Python. It extracts the dominant eigenvector and maps it
 * to the x, y, z, w members of the Quaternion_t struct.
 * 
 * @author Aditya (WhereSat Team)
 */

#include "quest.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define JACOBI_MAX_ITERATIONS 50
#define JACOBI_EPSILON 1e-7f

/**
 * @brief Robust Symmetric 4x4 Jacobi Eigensolver.
 */
static void jacobi_4x4(float A[4][4], float V[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            V[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int iter = 0; iter < JACOBI_MAX_ITERATIONS; iter++) {
        int p = 0, q = 1;
        float max_off_diag = fabsf(A[0][1]);
        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                if (fabsf(A[i][j]) > max_off_diag) {
                    max_off_diag = fabsf(A[i][j]);
                    p = i; q = j;
                }
            }
        }

        if (max_off_diag < JACOBI_EPSILON) break;

        float h = A[q][q] - A[p][p];
        float t;
        if (fabsf(h) + fabsf(A[p][q]) == fabsf(h)) {
            t = A[p][q] / h;
        } else {
            float theta = 0.5f * h / A[p][q];
            t = 1.0f / (fabsf(theta) + sqrtf(1.0f + theta * theta));
            if (theta < 0) t = -t;
        }

        float c = 1.0f / sqrtf(1.0f + t * t);
        float s = t * c;
        float tau = s / (1.0f + c);
        float g = t * A[p][q];

        A[p][p] -= g;
        A[q][q] += g;
        A[p][q] = 0.0f;
        A[q][p] = 0.0f;

        for (int i = 0; i < 4; i++) {
            if (i != p && i != q) {
                float g_p = A[i][p];
                float g_q = A[i][q];
                A[i][p] = g_p - s * (g_q + tau * g_p);
                A[p][i] = A[i][p];
                A[i][q] = g_q + s * (g_p - tau * g_q);
                A[q][i] = A[i][q];
            }
        }

        for (int i = 0; i < 4; i++) {
            float v_p = V[i][p];
            float v_q = V[i][q];
            V[i][p] = v_p - s * (v_q + tau * v_p);
            V[i][q] = v_q + s * (v_p - tau * v_q);
        }
    }
}

Quaternion_t quest_compute(QUEST_Input_t *input) {
    /* Guard: Minimum 2 stars required */
    if (input->count < 2) {
        return (Quaternion_t){0.0f, 0.0f, 0.0f, 1.0f};
    }

    float B[3][3] = {{0}};
    for (int k = 0; k < input->count; k++) {
        float w = input->weights[k];
        Vector3_t r = input->reference_v[k];
        Vector3_t b = input->body_v[k];
        B[0][0] += w * r.x * b.x; B[0][1] += w * r.x * b.y; B[0][2] += w * r.x * b.z;
        B[1][0] += w * r.y * b.x; B[1][1] += w * r.y * b.y; B[1][2] += w * r.y * b.z;
        B[2][0] += w * r.z * b.x; B[2][1] += w * r.z * b.y; B[2][2] += w * r.z * b.z;
    }

    float sigma = B[0][0] + B[1][1] + B[2][2];
    float S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) S[i][j] = B[i][j] + B[j][i];
    }

    float Z[3];
    Z[0] = B[1][2] - B[2][1];
    Z[1] = B[2][0] - B[0][2];
    Z[2] = B[0][1] - B[1][0];

    /* Construct K-Matrix (Vector-First: [x, y, z, w]) */
    float K[4][4] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = S[i][j];
            if (i == j) K[i][j] -= sigma;
        }
    }
    K[0][3] = Z[0]; K[1][3] = Z[1]; K[2][3] = Z[2];
    K[3][0] = Z[0]; K[3][1] = Z[1]; K[3][2] = Z[2];
    K[3][3] = sigma;

    float V[4][4];
    float K_work[4][4];
    memcpy(K_work, K, sizeof(K));
    jacobi_4x4(K_work, V);

    /* Find dominant eigenvalue */
    int max_idx = 0;
    float max_eig = K_work[0][0];
    for (int i = 1; i < 4; i++) {
        if (K_work[i][i] > max_eig) {
            max_eig = K_work[i][i];
            max_idx = i;
        }
    }

    /* Map Eigenvector [V0, V1, V2, V3] to Scalar-Last Struct [x, y, z, w] */
    Quaternion_t out;
    out.x = V[0][max_idx];
    out.y = V[1][max_idx];
    out.z = V[2][max_idx];
    out.w = V[3][max_idx]; // Scalar (w) is the 4th element

    /* Canonical form: Ensure scalar part (w) is positive */
    if (out.w < 0) {
        out.x = -out.x;
        out.y = -out.y;
        out.z = -out.z;
        out.w = -out.w;
    }

    return quat_normalize(out);
}
