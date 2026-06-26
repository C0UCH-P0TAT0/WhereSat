#include "star_matcher.h"

// ---------------------------------------------------------
// Internal Voting Structure (Static Memory)
// ---------------------------------------------------------
// We use this to tally votes for each observed star.
typedef struct {
    uint32_t hip_ids[MAX_TRACKED_HIPS];
    uint8_t counts[MAX_TRACKED_HIPS];
} VoteBox;

// ---------------------------------------------------------
// Helper: Add a vote to a specific local star's ballot box
// ---------------------------------------------------------
static void add_vote(VoteBox *box, uint32_t hip_id) {
    // 1. Check if this HIP ID is already in the box
    for (uint8_t i = 0; i < MAX_TRACKED_HIPS; i++) {
        if (box->hip_ids[i] == hip_id) {
            box->counts[i]++;
            return; // Vote added, we are done
        }
    }
    
    // 2. If it's not in the box, find an empty slot and add it
    for (uint8_t i = 0; i < MAX_TRACKED_HIPS; i++) {
        if (box->hip_ids[i] == 0) { // 0 means empty slot
            box->hip_ids[i] = hip_id;
            box->counts[i] = 1;
            return;
        }
    }
    // If the box is full (more than MAX_TRACKED_HIPS unique candidates), 
    // we just ignore the vote. True matches will get voted in early anyway.
}

// ---------------------------------------------------------
// Main Function: Cross-Reference and Match
// ---------------------------------------------------------
void match_stars(const ObservedTriangle *triangles, uint16_t num_triangles, 
                 uint8_t num_observed_stars, MatchedStar *out_matches) {
    
    // Create our ballot boxes (one for each star the camera sees)
    VoteBox ballot_boxes[MAX_OBSERVED_STARS] = {0}; // Initializes all to 0

    CandidateMatch candidates[MAX_CANDIDATES];
    uint8_t num_candidates;

    // ==========================================
    // PHASE 1: VOTING
    // ==========================================
    for (uint16_t t = 0; t < num_triangles; t++) {
        
        // Get all database candidates for this specific triangle
        find_candidate_triangles(&triangles[t], candidates, &num_candidates);

        // For every candidate we found...
        for (uint8_t c = 0; c < num_candidates; c++) {
            
            // We don't know exactly which local star matches which HIP ID yet,
            // so we give a vote to ALL 3 HIP IDs for ALL 3 local stars in this triangle.
            // The fake matches will scatter, but the TRUE match will stack up votes!
            for (uint8_t local_idx = 0; local_idx < 3; local_idx++) {
                uint8_t star_id = triangles[t].star_indices[local_idx];
                
                add_vote(&ballot_boxes[star_id], candidates[c].hips[0]);
                add_vote(&ballot_boxes[star_id], candidates[c].hips[1]);
                add_vote(&ballot_boxes[star_id], candidates[c].hips[2]);
            }
        }
    }

    // ==========================================
    // PHASE 2: COUNTING THE VOTES
    // ==========================================
    for (uint8_t s = 0; s < num_observed_stars; s++) {
        out_matches[s].local_id = s;
        out_matches[s].is_matched = false;
        out_matches[s].hip_id = 0;
        out_matches[s].vote_count = 0;

        uint8_t max_votes = 0;
        uint32_t best_hip = 0;

        // Look through the ballot box for this star to find the winner
        for (uint8_t i = 0; i < MAX_TRACKED_HIPS; i++) {
            if (ballot_boxes[s].counts[i] > max_votes) {
                max_votes = ballot_boxes[s].counts[i];
                best_hip = ballot_boxes[s].hip_ids[i];
            }
        }

        // Did the winner get enough votes to be considered a True Match?
        if (max_votes >= MIN_VOTES_REQUIRED) {
            out_matches[s].is_matched = true;
            out_matches[s].hip_id = best_hip;
            out_matches[s].vote_count = max_votes;
        }
    }
}