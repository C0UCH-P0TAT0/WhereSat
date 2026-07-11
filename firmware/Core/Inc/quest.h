/**
 * @file quest.h
 * @brief QUEST (Quaternion Estimator) algorithm for attitude determination.
 *
 * @author Aditya (WhereSat Team)
 */

#ifndef INC_QUEST_H_
#define INC_QUEST_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "quaternion.h"
#include <stdint.h>

/* Increased from 10 to 16 to safely handle up to 12 stars and prevent buffer overflow */
#define MAX_QUEST_VECTORS 16

/**
 * @brief Structure to hold observation and reference vector pairs.
 */
typedef struct {
    Vector3_t body_v[MAX_QUEST_VECTORS];      // Measured (from Camera / Star Tracker)
    Vector3_t reference_v[MAX_QUEST_VECTORS]; // Reference (from Star Catalog)
    float weights[MAX_QUEST_VECTORS];         // Weight of each measurement (typically 1.0)
    int count;                                // Number of valid vectors in this frame
} QUEST_Input_t;

/**
 * @brief Computes the optimal attitude quaternion using Davenport's q-method.
 * 
 * @param input Pointer to the populated QUEST input structure.
 * @return Quaternion_t The estimated attitude quaternion. Returns {NAN, NAN, NAN, NAN} on failure.
 */
Quaternion_t quest_compute(QUEST_Input_t *input);

#ifdef __cplusplus
}
#endif

#endif /* INC_QUEST_H_ */