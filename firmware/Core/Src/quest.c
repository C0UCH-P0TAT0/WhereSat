/**
 * @file quest.c
 * @brief QUEST (Quaternion Estimator) implementation using Davenport's q-method.
 *
 * @author Aditya (WhereSat Team)
 */

#include "quest.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* --- Configuration --- */
#define JACOBI_MAX_ITERATIONS 50
#define JACOBI_EPSILON        1e-7f
#define QUEST_MIN_STARS       2

/* Toggle this to 0 if the satellite spins the wrong way in HIL.
   1 = Body-to-ECI, 0 = ECI-to-Body */
#define OUTPUT_BODY_TO_ECI    1 

/* --- Private Function Prototypes --- */
static bool jacobi_4x4(float A[4][4], float V[4][4]);

/**
 * @brief Robust Symmetric 4x4 Jacobi Eigensolver.
 */
static bool jacobi_4x4(float A[4][4], float V[4][4]) {
    // Initialize V as Identity Matrix
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            V[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int iter = 0; iter < JACOBI_MAX_ITERATIONS; iter++) {
        int p = 0, q = 1;
        float max_off_diag = fabsf(A[0][1]);
        
        if (isnan(max_off_diag)) return false;

        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                float val = fabsf(A[i][j]);
                if (isnan(val)) return false;
                
                if (val > max_off_diag) {
                    max_off_diag = val;
                    p = i;
                    q = j;
                }
            }
        }

        if (max_off_diag < JACOBI_EPSILON) return true;

        float h = A[q][q] - A[p][p];
        float t;
        if (fabsf(h) + max_off_diag == fabsf(h)) {
            t = A[p][q] / h;
        } else {
            float theta = 0.5f * h / A[p][q];
            t = 1.0f / (fabsf(theta) + sqrtf(1.0f + theta * theta));
            if (theta < 0.0f) t = -t;
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
    
    return false;
}

/**
 * @brief Computes the optimal attitude quaternion using Davenport's q-method.
 */
Quaternion_t quest_compute(QUEST_Input_t *input) {
    /* 0. Input Bounds Guard */
    if (input->count > MAX_QUEST_VECTORS) {
        printf("QUEST ERROR: Input count (%d) exceeds max capacity (%d).\r\n", input->count, MAX_QUEST_VECTORS);
        return (Quaternion_t){NAN, NAN, NAN, NAN};
    }

    /* 1. Minimum stars required for a valid solution */
    if (input->count < QUEST_MIN_STARS) {
        printf("QUEST WARNING: Not enough stars matched (%d/%d).\r\n", input->count, QUEST_MIN_STARS);
        return (Quaternion_t){NAN, NAN, NAN, NAN};
    }

    float B[3][3] = {{0.0f}};

    /* 2. Build Attitude Profile Matrix B = Σ w * r * b^T */
    for (int k = 0; k < input->count; k++) {
        float w = input->weights[k];
        Vector3_t r = input->reference_v[k];
        Vector3_t b = input->body_v[k];

        // Input payload corruption guard explicitly returning NAN
        if (isnan(r.x) || isnan(r.y) || isnan(r.z) || isnan(b.x) || isnan(b.y) || isnan(b.z)) {
            printf("QUEST ERROR: NaN detected in input vectors. Index: %d\r\n", k);
            return (Quaternion_t){NAN, NAN, NAN, NAN};
        }

        B[0][0] += w * r.x * b.x; B[0][1] += w * r.x * b.y; B[0][2] += w * r.x * b.z;
        B[1][0] += w * r.y * b.x; B[1][1] += w * r.y * b.y; B[1][2] += w * r.y * b.z;
        B[2][0] += w * r.z * b.x; B[2][1] += w * r.z * b.y; B[2][2] += w * r.z * b.z;
    }

    /* 3. Compute S, Sigma (trace), and Z vector */
    float sigma = B[0][0] + B[1][1] + B[2][2];

    float S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = B[i][j] + B[j][i];
        }
    }

    float Z[3];
    Z[0] = B[1][2] - B[2][1];
    Z[1] = B[2][0] - B[0][2];
    Z[2] = B[0][1] - B[1][0];

    /* 4. Construct Davenport K-Matrix */
    float K[4][4] = {{0.0f}};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = S[i][j];
            if (i == j) K[i][j] -= sigma;
        }
    }

    K[0][3] = Z[0]; K[1][3] = Z[1]; K[2][3] = Z[2];
    K[3][0] = Z[0]; K[3][1] = Z[1]; K[3][2] = Z[2];
    K[3][3] = sigma;

    /* 5. Solve for Eigenvalues/Eigenvectors */
    float V[4][4];
    float K_work[4][4];
    memcpy(K_work, K, sizeof(K));
    
    if (!jacobi_4x4(K_work, V)) {
        printf("QUEST ERROR: Jacobi eigensolver failed to converge or hit NaN.\r\n");
        return (Quaternion_t){NAN, NAN, NAN, NAN};
    }

    /* 6. Find index of the largest Eigenvalue */
    int max_idx = 0;
    float max_eig = K_work[0][0];
    for (int i = 1; i < 4; i++) {
        if (K_work[i][i] > max_eig) {
            max_eig = K_work[i][i];
            max_idx = i;
        }
    }

    /* 7. Map Eigenvector to Scalar-Last Quaternion */
    Quaternion_t out;
    out.x = V[0][max_idx];
    out.y = V[1][max_idx];
    out.z = V[2][max_idx];
    out.w = V[3][max_idx];

    /* 8. Final Math Guard */
    if (isnan(out.x) || isnan(out.y) || isnan(out.z) || isnan(out.w) ||
        isinf(out.x) || isinf(out.y) || isinf(out.z) || isinf(out.w)) {
        printf("QUEST ERROR: Final output quaternion contains NaN or Inf.\r\n");
        return (Quaternion_t){NAN, NAN, NAN, NAN};
    }

    /* 9. Frame Inversion Toggle via Conjugation */
    #if OUTPUT_BODY_TO_ECI
        out.x = -out.x;
        out.y = -out.y;
        out.z = -out.z;
    #endif

    /* 10. Canonical form: Ensure scalar part (w) is positive */
    if (out.w < 0.0f) {
        out.x = -out.x;
        out.y = -out.y;
        out.z = -out.z;
        out.w = -out.w;
    }

    return quat_normalize(out);
}