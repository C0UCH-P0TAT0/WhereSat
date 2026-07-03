/**
 * @file quest.h
 * @brief QUEST (Quaternion Estimator) algorithm for attitude determination.
 *
 * @author Aditya (WhereSat Team)
 */

#ifndef INC_QUEST_H_
#define INC_QUEST_H_

#include "quaternion.h"

#define MAX_QUEST_VECTORS 10

/**
 * @brief Structure to hold observation and reference vector pairs.
 */
typedef struct {
    Vector3_t body_v[MAX_QUEST_VECTORS];      // Measured (from Camera)
    Vector3_t reference_v[MAX_QUEST_VECTORS]; // Reference (from Catalog)
    float weights[MAX_QUEST_VECTORS];
    int count;
} QUEST_Input_t;

Quaternion_t quest_compute(QUEST_Input_t *input);

#endif /* INC_QUEST_H_ */
