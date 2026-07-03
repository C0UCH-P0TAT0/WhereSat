/**
 * @file quest.c
 * @brief QUEST / Davenport q-method implementation
 */

#include "quest.h"
#include <math.h>
#include <string.h>

#define POWER_ITERATIONS 20

Quaternion_t quest_compute(QUEST_Input_t *input)
{
    float B[3][3] = {{0}};

    // ----------------------------------------------------
    // Build B = Σ w * r * bᵀ
    // r = reference (ECI)
    // b = measured body vector
    // ----------------------------------------------------
    for(int k=0;k<input->count;k++)
    {
        float w = input->weights[k];

        Vector3_t r = input->reference_v[k];
        Vector3_t b = input->body_v[k];

        B[0][0] += w*r.x*b.x;
        B[0][1] += w*r.x*b.y;
        B[0][2] += w*r.x*b.z;

        B[1][0] += w*r.y*b.x;
        B[1][1] += w*r.y*b.y;
        B[1][2] += w*r.y*b.z;

        B[2][0] += w*r.z*b.x;
        B[2][1] += w*r.z*b.y;
        B[2][2] += w*r.z*b.z;
    }

    //--------------------------------------------
    // sigma
    //--------------------------------------------
    float sigma =
        B[0][0] +
        B[1][1] +
        B[2][2];

    //--------------------------------------------
    // S = B + Bᵀ
    //--------------------------------------------
    float S[3][3];

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
            S[i][j]=B[i][j]+B[j][i];
    }

    //--------------------------------------------
    // Z
    //--------------------------------------------
    float Z[3];

    Z[0]=B[1][2]-B[2][1];
    Z[1]=B[2][0]-B[0][2];
    Z[2]=B[0][1]-B[1][0];

    //--------------------------------------------
    // Davenport K matrix
    //--------------------------------------------
    float K[4][4];

    K[0][0]=sigma;
    K[0][1]=Z[0];
    K[0][2]=Z[1];
    K[0][3]=Z[2];

    K[1][0]=Z[0];
    K[2][0]=Z[1];
    K[3][0]=Z[2];

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            float val=S[i][j];

            if(i==j)
                val-=sigma;

            K[i+1][j+1]=val;
        }
    }

    //--------------------------------------------
    // Power Iteration
    //--------------------------------------------
    float q[4]={1.0f,0,0,0};

    for(int iter=0;iter<POWER_ITERATIONS;iter++)
    {
        float nq[4]={0};

        for(int i=0;i<4;i++)
        {
            for(int j=0;j<4;j++)
                nq[i]+=K[i][j]*q[j];
        }

        float norm=sqrtf(
            nq[0]*nq[0]+
            nq[1]*nq[1]+
            nq[2]*nq[2]+
            nq[3]*nq[3]);

        if(norm<1e-8f)
            break;

        for(int i=0;i<4;i++)
            q[i]=nq[i]/norm;
    }

    Quaternion_t out;

    // scalar-first
    out.q0=q[0];
    out.q1=q[1];
    out.q2=q[2];
    out.q3=q[3];

    return quat_normalize(out);
}
