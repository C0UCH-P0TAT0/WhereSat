/**
 * @file star_matcher.c
 * @brief Star identification logic using a voting-based pyramid/triangle approach.
 */

#include "star_matcher.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// ---------------------------------------------------------
// Internal Voting Structures
// ---------------------------------------------------------
typedef struct {
    uint32_t hip_ids[MAX_TRACKED_HIPS];
    uint8_t counts[MAX_TRACKED_HIPS];
} VoteBox;

typedef struct {
    uint8_t local_id;
    uint32_t best_hip;
    uint8_t best_votes;
    uint32_t runner_up_hip;
    uint8_t runner_up_votes;
} VoteResult;

#define EMPTY_SLOT 0xFFFFFFFF

// ---------------------------------------------------------
// Helper: Add a vote to a specific local star's ballot box
// ---------------------------------------------------------
static void add_vote(VoteBox *box, uint32_t hip_id) {
    if (hip_id == 0) return;

    for (uint8_t i = 0; i < MAX_TRACKED_HIPS; i++) {
        if (box->hip_ids[i] == hip_id) {
            box->counts[i]++;
            return; 
        }
    }
    
    for (uint8_t i = 0; i < MAX_TRACKED_HIPS; i++) {
        if (box->hip_ids[i] == EMPTY_SLOT) { 
            box->hip_ids[i] = hip_id;
            box->counts[i] = 1;
            return;
        }
    }
}

/**
 * @brief Matches observed stars to catalog HIP IDs using a voting scheme.
 */
void match_stars(const ObservedStar *observed_stars, const ObservedTriangle *triangles, 
                 uint16_t num_triangles, uint8_t num_observed_stars, 
                 MatchedStar *out_matches) {
    
    VoteBox ballot_boxes[MAX_OBSERVED_STARS];
    
    // Initialize the ballot boxes
    for (uint8_t i = 0; i < MAX_OBSERVED_STARS; i++) {
        for (uint8_t j = 0; j < MAX_TRACKED_HIPS; j++) {
            ballot_boxes[i].hip_ids[j] = EMPTY_SLOT;
            ballot_boxes[i].counts[j] = 0;
        }
    }

    CandidateMatch candidates[MAX_CANDIDATES];
    uint8_t num_candidates;

// ==========================================
    // PHASE 1: DETERMINISTIC VERTEX MAPPING
    // ==========================================
    const uint8_t perms[6][3] = {
        {0, 1, 2}, {0, 2, 1}, {1, 0, 2}, 
        {1, 2, 0}, {2, 0, 1}, {2, 1, 0}
    };

    for (uint16_t t = 0; t < num_triangles; t++) {
        find_candidate_triangles(&triangles[t], candidates, &num_candidates);
        if (num_candidates == 0) continue;

        uint8_t id0 = triangles[t].star_indices[0];
        uint8_t id1 = triangles[t].star_indices[1];
        uint8_t id2 = triangles[t].star_indices[2];

        // Bounds guard
        if (id0 >= MAX_OBSERVED_STARS || id1 >= MAX_OBSERVED_STARS || id2 >= MAX_OBSERVED_STARS) continue;

        // Compute local dot products once per triangle
        float dot_L01 = observed_stars[id0].x * observed_stars[id1].x + 
                        observed_stars[id0].y * observed_stars[id1].y + 
                        observed_stars[id0].z * observed_stars[id1].z;
                        
        float dot_L12 = observed_stars[id1].x * observed_stars[id2].x + 
                        observed_stars[id1].y * observed_stars[id2].y + 
                        observed_stars[id1].z * observed_stars[id2].z;
                        
        float dot_L02 = observed_stars[id0].x * observed_stars[id2].x + 
                        observed_stars[id0].y * observed_stars[id2].y + 
                        observed_stars[id0].z * observed_stars[id2].z;

        for (uint8_t c = 0; c < num_candidates; c++) {
            Vector3_t u[3];
            catalog_get_star_vector(candidates[c].hips[0], &u[0]);
            catalog_get_star_vector(candidates[c].hips[1], &u[1]);
            catalog_get_star_vector(candidates[c].hips[2], &u[2]);

            int best_p = -1;
            float best_err = 999.0f;

            // Test all 6 permutations to find the true geometric mapping
            for (int p = 0; p < 6; p++) {
                uint8_t i0 = perms[p][0];
                uint8_t i1 = perms[p][1];
                uint8_t i2 = perms[p][2];

                float d01 = u[i0].x * u[i1].x + u[i0].y * u[i1].y + u[i0].z * u[i1].z;
                float d12 = u[i1].x * u[i2].x + u[i1].y * u[i2].y + u[i1].z * u[i2].z;
                float d02 = u[i0].x * u[i2].x + u[i0].y * u[i2].y + u[i0].z * u[i2].z;

                float e1 = fabsf(dot_L01 - d01);
                float e2 = fabsf(dot_L12 - d12);
                float e3 = fabsf(dot_L02 - d02);

                if (e1 < GEOMETRIC_TOLERANCE_DOT && e2 < GEOMETRIC_TOLERANCE_DOT && e3 < GEOMETRIC_TOLERANCE_DOT) {
                    float sum_err = e1 + e2 + e3;
                    if (sum_err < best_err) {
                        best_err = sum_err;
                        best_p = p;
                    }
                }
            }

            // Only cast votes if a valid permutation was found
            if (best_p != -1) {
                add_vote(&ballot_boxes[id0], candidates[c].hips[perms[best_p][0]]);
                add_vote(&ballot_boxes[id1], candidates[c].hips[perms[best_p][1]]);
                add_vote(&ballot_boxes[id2], candidates[c].hips[perms[best_p][2]]);
            }
        }
    }
    // ==========================================
    // PHASE 2: EXTRACT BEST & RUNNER-UP
    // ==========================================
    VoteResult results[MAX_OBSERVED_STARS];
    
    for (uint8_t s = 0; s < num_observed_stars; s++) {
        results[s].local_id = s;
        results[s].best_hip = 0;
        results[s].best_votes = 0;
        results[s].runner_up_hip = 0;
        results[s].runner_up_votes = 0;

        for (uint8_t i = 0; i < MAX_TRACKED_HIPS; i++) {
            uint8_t v = ballot_boxes[s].counts[i];
            uint32_t h = ballot_boxes[s].hip_ids[i];
            
            if (h == EMPTY_SLOT || v == 0) continue;

            if (v > results[s].best_votes) {
                results[s].runner_up_votes = results[s].best_votes;
                results[s].runner_up_hip = results[s].best_hip;
                results[s].best_votes = v;
                results[s].best_hip = h;
            } else if (v > results[s].runner_up_votes) {
                results[s].runner_up_votes = v;
                results[s].runner_up_hip = h;
            }
        }
    }

    // ==========================================
    // PHASE 3: GREEDY SORT (Strongest First)
    // ==========================================
    for (uint8_t i = 0; i < num_observed_stars - 1; i++) {
        for (uint8_t j = i + 1; j < num_observed_stars; j++) {
            if (results[j].best_votes > results[i].best_votes) {
                VoteResult temp = results[i];
                results[i] = results[j];
                results[j] = temp;
            }
        }
    }

    // ==========================================
    // PHASE 4: ASSIGNMENT & UNIQUENESS LOCK
    // ==========================================
    uint32_t assigned_hips[MAX_OBSERVED_STARS];
    uint8_t assigned_count = 0;

    // Pre-clear output array
    for (uint8_t s = 0; s < num_observed_stars; s++) {
        out_matches[s].local_id = s;
        out_matches[s].is_matched = false;
        out_matches[s].hip_id = 0;
        out_matches[s].vote_count = 0;
    }

    printf("\r\n--- STAR MATCHING VOTES ---\r\n");
    for (uint8_t i = 0; i < num_observed_stars; i++) {
        uint8_t s = results[i].local_id;
        
        if (results[i].best_votes >= MIN_VOTES_REQUIRED) {
            // Uniqueness Check
            bool is_taken = false;
            for (uint8_t j = 0; j < assigned_count; j++) {
                if (assigned_hips[j] == results[i].best_hip) {
                    is_taken = true;
                    break;
                }
            }

            if (!is_taken) {
                out_matches[s].is_matched = true;
                out_matches[s].hip_id = results[i].best_hip;
                out_matches[s].vote_count = results[i].best_votes;
                assigned_hips[assigned_count++] = results[i].best_hip;
                printf("Star %d -> HIP %lu (%d votes) [LOCKED]\r\n", s, results[i].best_hip, results[i].best_votes);
            } else {
                printf("Star %d -> HIP %lu REJECTED (Collision)\r\n", s, results[i].best_hip);
            }
        }
    }

    // ==========================================
    // PHASE 5: GEOMETRIC VERIFICATION (Consensus)
    // ==========================================
    Vector3_t cat_vecs[MAX_OBSERVED_STARS];
    uint8_t consensus_score[MAX_OBSERVED_STARS] = {0};

    // 1. Fetch catalog vectors for all tentative assignments
    for (uint8_t i = 0; i < num_observed_stars; i++) {
        if (out_matches[i].is_matched) {
            catalog_get_star_vector(out_matches[i].hip_id, &cat_vecs[i]);
        }
    }

    // 2. Pairwise dot product comparison
    for (uint8_t i = 0; i < num_observed_stars - 1; i++) {
        if (!out_matches[i].is_matched) continue;
        
        for (uint8_t j = i + 1; j < num_observed_stars; j++) {
            if (!out_matches[j].is_matched) continue;
            
            // Use the observed_stars pointer passed from main.c
            float dot_obs = observed_stars[i].x * observed_stars[j].x + 
                            observed_stars[i].y * observed_stars[j].y + 
                            observed_stars[i].z * observed_stars[j].z;
                            
            float dot_cat = cat_vecs[i].x * cat_vecs[j].x + 
                            cat_vecs[i].y * cat_vecs[j].y + 
                            cat_vecs[i].z * cat_vecs[j].z;

            if (i == 0 && j == 1) {
                printf("DEBUG PAIR 0-1 | OBS DOT: %.6f | CAT DOT: %.6f | DIFF: %.6f\r\n", 
                dot_obs, dot_cat, fabsf(dot_obs - dot_cat));
            }

            if (fabsf(dot_obs - dot_cat) < GEOMETRIC_TOLERANCE_DOT) {
                consensus_score[i]++;
                consensus_score[j]++;
            }
        }   
    }

    // 3. Reject outliers lacking consensus
    uint8_t required_consensus = (assigned_count > 3) ? (assigned_count / 2) : 1;

    printf("\r\n--- GEOMETRIC VERIFICATION ---\r\n");
    for (uint8_t i = 0; i < num_observed_stars; i++) {
        if (out_matches[i].is_matched) {
            if (consensus_score[i] < required_consensus) {
                printf("REJECTED: Star %d (HIP %lu) Consensus %d/%d\r\n", 
                       i, out_matches[i].hip_id, consensus_score[i], assigned_count - 1);
                out_matches[i].is_matched = false;
            } else {
                printf("VERIFIED: Star %d (HIP %lu) Consensus %d/%d\r\n", 
                       i, out_matches[i].hip_id, consensus_score[i], assigned_count - 1);
            }
        }
    }
    printf("------------------------------\r\n");
}