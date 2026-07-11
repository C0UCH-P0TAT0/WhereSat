/**
 * @file mekf.c
 * @brief 6-State MEKF Implementation (Silent Math Engine).
 *
 * Features: Scalar-last convention, Joseph-form update, symmetry enforcement,
 * and singularity-protected 3x3 matrix inversion.
 *
 * @author Aditya (WhereSat Team)
 */

#include "mekf.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define MEKF_SMALL_ANGLE 1e-8f

/* --- Private Matrix Helpers --- */

/**
 * @brief Singularity-protected 3x3 Matrix Inversion.
 * Returns false if the matrix is singular (determinant near zero).
 */
static bool mat3_inv(float m[3][3], float inv[3][3]) {
    float det = m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1]) -
                m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0]) +
                m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]);

    if (fabsf(det) < 1e-12f) return false;

    float invDet = 1.0f / det;
    inv[0][0] = (m[1][1]*m[2][2] - m[1][2]*m[2][1]) * invDet;
    inv[0][1] = (m[0][2]*m[2][1] - m[0][1]*m[2][2]) * invDet;
    inv[0][2] = (m[0][1]*m[1][2] - m[0][2]*m[1][1]) * invDet;
    inv[1][0] = (m[1][2]*m[2][0] - m[1][0]*m[2][2]) * invDet;
    inv[1][1] = (m[0][0]*m[2][2] - m[0][2]*m[2][0]) * invDet;
    inv[1][2] = (m[1][0]*m[0][2] - m[0][0]*m[1][2]) * invDet;
    inv[2][0] = (m[1][0]*m[2][1] - m[1][1]*m[2][0]) * invDet;
    inv[2][1] = (m[2][0]*m[0][1] - m[0][0]*m[2][1]) * invDet;
    inv[2][2] = (m[0][0]*m[1][1] - m[1][0]*m[0][1]) * invDet;
    return true;
}

/**
 * @brief Enforces symmetry on the 6x6 covariance matrix to prevent numerical drift.
 */
static void mekf_enforce_symmetry(MEKF_t *f) {
    for (int i = 0; i < 6; i++) {
        for (int j = i + 1; j < 6; j++) {
            float avg = 0.5f * (f->P[i][j] + f->P[j][i]);
            f->P[i][j] = avg;
            f->P[j][i] = avg;
        }
    }
}

void mekf_init(MEKF_t *f, Quaternion_t initial_q) {
    f->q = quat_normalize(initial_q);
    memset(f->beta, 0, sizeof(f->beta));
    memset(f->P, 0, sizeof(f->P));
    
    // Initial uncertainties
    for(int i=0; i<3; i++) f->P[i][i] = 0.1f;
    for(int i=3; i<6; i++) f->P[i][i] = 0.001f;

    // Process noise parameters
    f->Q_v = 1e-3f; 
    f->Q_u = 3e-10f;
}

void mekf_predict(MEKF_t *f, Vector3_t omega_meas, float dt) {
    // 1. Bias Correction
    Vector3_t w = {omega_meas.x - f->beta[0], omega_meas.y - f->beta[1], omega_meas.z - f->beta[2]};
    float w_mag = sqrtf(w.x*w.x + w.y*w.y + w.z*w.z);

    // 2. Quaternion Propagation (Scalar-Last)
    Quaternion_t dq;
    if (w_mag < MEKF_SMALL_ANGLE) {
        dq.w = 1.0f;
        dq.x = 0.5f * w.x * dt;
        dq.y = 0.5f * w.y * dt;
        dq.z = 0.5f * w.z * dt;
    } else {
        float theta = w_mag * dt;
        float scale = sinf(theta * 0.5f) / w_mag;
        dq.w = cosf(theta * 0.5f);
        dq.x = w.x * scale;
        dq.y = w.y * scale;
        dq.z = w.z * scale;
    }
    f->q = quat_normalize(quat_multiply(f->q, dq));

    // 3. Covariance Prediction: P = Phi*P*Phi^T + Q*dt
    float Phi[6][6] = {0};
    for(int i=0; i<6; i++) Phi[i][i] = 1.0f;
    Phi[0][1] =  w.z*dt; Phi[0][2] = -w.y*dt;
    Phi[1][0] = -w.z*dt; Phi[1][2] =  w.x*dt;
    Phi[2][0] =  w.y*dt; Phi[2][1] = -w.x*dt;
    for(int i=0; i<3; i++) Phi[i][i+3] = -dt;

    float tmp[6][6] = {0};
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            for(int k=0; k<6; k++) tmp[i][j] += Phi[i][k] * f->P[k][j];
        }
    }
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            float val = 0;
            for(int k=0; k<6; k++) val += tmp[i][k] * Phi[j][k];
            f->P[i][j] = val;
        }
        // Timestep-dependent process noise
        if (i < 3) f->P[i][i] += f->Q_v * dt;
        else       f->P[i][i] += f->Q_u * dt;
    }
    mekf_enforce_symmetry(f);
}

void mekf_update(MEKF_t *f, Quaternion_t q_meas, float r_noise) {
    // 1. Error Quaternion (Scalar-Last)
    Quaternion_t q_err = quat_multiply(quat_conjugate(f->q), q_meas);
    if (q_err.w < 0) { q_err.x*=-1; q_err.y*=-1; q_err.z*=-1; q_err.w*=-1; }
    float alpha[3] = {2.0f * q_err.x, 2.0f * q_err.y, 2.0f * q_err.z};

    // 2. Kalman Gain
    float S[3][3], S_inv[3][3];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) S[i][j] = f->P[i][j] + (i==j ? r_noise : 0.0f);
    }
    
    if (!mat3_inv(S, S_inv)) {
        printf("Step 6: MEKF UPDATE FAILED (S matrix singular)\r\n");
        return;
    }

    float K[6][3] = {0};
    for(int i=0; i<6; i++) {
        for(int j=0; j<3; j++) {
            for(int k=0; k<3; k++) K[i][j] += f->P[i][k] * S_inv[k][j];
        }
    }

    // 3. State Update
    float dx[6] = {0};
    for(int i=0; i<6; i++) {
        for(int j=0; j<3; j++) dx[i] += K[i][j] * alpha[j];
    }

    Quaternion_t dq = {dx[0]*0.5f, dx[1]*0.5f, dx[2]*0.5f, 1.0f};
    f->q = quat_normalize(quat_multiply(f->q, quat_normalize(dq)));
    for(int i=0; i<3; i++) f->beta[i] += dx[i+3];

    // 4. Joseph Form Covariance Update
    float IKH[6][6] = {0};
    for(int i=0; i<6; i++) IKH[i][i] = 1.0f;
    for(int i=0; i<6; i++) {
        for(int j=0; j<3; j++) IKH[i][j] -= K[i][j];
    }

    float tmpP[6][6] = {0};
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            for(int k=0; k<6; k++) tmpP[i][j] += IKH[i][k] * f->P[k][j];
        }
    }
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            float val = 0;
            for(int k=0; k<6; k++) val += tmpP[i][k] * IKH[j][k];
            f->P[i][j] = val;
        }
    }
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            for(int k=0; k<3; k++) f->P[i][j] += K[i][k] * r_noise * K[j][k];
        }
    }
    mekf_enforce_symmetry(f);
}