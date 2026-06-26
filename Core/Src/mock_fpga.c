/**
 * @file mock_fpga.c
 * @brief Simulated FPGA data provider for software-in-the-loop testing.
 *
 * This file populates FPGA_Packet_t structures with known star patterns
 * to test the downstream triangle building and catalog search algorithms.
 *
 * @author Aditya (WhereSat Team)
 */

#include "fpga_interface.h"

/**
 * @brief Fills a packet with mock star data.
 * @param packet Pointer to the packet to populate.
 */
void load_test_centroids(FPGA_Packet_t *packet) {
    packet->header = FPGA_PACKET_HEADER;
    packet->count = 3; // A simple triangle

    // Star 1
    packet->centroids[0].x = 640.0f;
    packet->centroids[0].y = 480.0f;

    // Star 2
    packet->centroids[1].x = 700.0f;
    packet->centroids[1].y = 480.0f;

    // Star 3
    packet->centroids[2].x = 640.0f;
    packet->centroids[2].y = 550.0f;

    // Calculate dummy checksum
    uint8_t checksum = 0;
    uint8_t *ptr = (uint8_t*)packet;
    for(int i=0; i < sizeof(FPGA_Packet_t)-1; i++) {
        checksum ^= ptr[i];
    }
    packet->checksum = checksum;
}
