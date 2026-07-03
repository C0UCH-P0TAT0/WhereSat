/**
 * @file quest.c
 * @brief QUEST implementation with a robust Jacobi Eigensolver.
 * 
 * This version uses the standard Jacobi rotation algorithm to ensure 
 * numerical stability and orthogonality of eigenvectors.
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
 * 
 * This implementation uses the stable "Numerical Recipes" update logic.
 * A is the symmetric matrix (modified to diagonal), V is the eigenvector matrix.
 */
static void jacobi_4x4(float A[4][4], float V[4][4]) {
    // Initialize V as Identity
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            V[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int iter = 0; iter < JACOBI_MAX_ITERATIONS; iter++) {
        // 1. Find the largest off-diagonal element
        int p = 0, q = 1;
        float max_off_diag = fabsf(A[0][1]);
        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                if (fabsf(A[i][j]) > max_off_diag) {
                    max_off_diag = fabsf(A[i][j]);
                    p = i;
                    q = j;
                }
            }
        }

        // Convergence check
        if (max_off_diag < JACOBI_EPSILON) break;

        // 2. Compute rotation parameters
        float h = A[q][q] - A[p][p];
        float t;
        if (fabsf(h) + fabsf(A[p][q]) == fabsf(h)) {
            t = A[p][q] / h; // t = tan(phi)
        } else {
            float theta = 0.5f * h / A[p][q];
            t = 1.0f / (fabsf(theta) + sqrtf(1.0f + theta * theta));
            if (theta < 0) t = -t;
        }

        float c = 1.0f / sqrtf(1.0f + t * t);
        float s = t * c;
        float tau = s / (1.0f + c);
        float g = t * A[p][q];

        // 3. Update diagonal and zero out A[p][q]
        A[p][p] -= g;
        A[q][q] += g;
        A[p][q] = 0.0f;
        A[q][p] = 0.0f;

        // 4. Update off-diagonal elements (p and q rows/cols)
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

        // 5. Update Eigenvector matrix V
        for (int i = 0; i < 4; i++) {
            float v_p = V[i][p];
            float v_q = V[i][q];
            V[i][p] = v_p - s * (v_q + tau * v_p);
            V[i][q] = v_q + s * (v_p - tau * v_q);
        }
    }
}

Quaternion_t quest_compute(QUEST_Input_t *input) {
    if (input->count < 2) {
        return (Quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
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

    /* Convergence Check: Print Eigenvalues */
    printf("\nEigenvalues (Diagonal of K_work):\n");
    for(int i = 0; i < 4; i++) {
        printf("  L[%d]: % .6f\n", i, K_work[i][i]);
    }

    /* Find dominant eigenvalue */
    int max_idx = 0;
    float max_eig = K_work[0][0];
    for (int i = 1; i < 4; i++) {
        if (K_work[i][i] > max_eig) {
            max_eig = K_work[i][i];
            max_idx = i;
        }
    }

    /* Map [x, y, z, w] eigenvector to [w, x, y, z] quaternion */
    Quaternion_t out;
    out.q0 = V[3][max_idx]; // w
    out.q1 = V[0][max_idx]; // x
    out.q2 = V[1][max_idx]; // y
    out.q3 = V[2][max_idx]; // z

    if (out.q0 < 0) {
        out.q0 = -out.q0; out.q1 = -out.q1; out.q2 = -out.q2; out.q3 = -out.q3;
    }

    return quat_normalize(out);
}