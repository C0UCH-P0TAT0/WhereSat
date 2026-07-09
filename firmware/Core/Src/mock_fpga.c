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
#include <string.h>

/**
 * @brief Fills a packet with mock star data.
 * @param packet Pointer to the packet to populate.
 */
void load_test_centroids(FPGA_Packet_t *packet) {
    // 1. Clear the entire struct to zero
    memset(packet, 0, sizeof(FPGA_Packet_t));
    packet->header = FPGA_PACKET_HEADER;
    packet->count = 5;

    packet->centroids[0].x = 728.159f;  packet->centroids[0].y = 33.526f;
    packet->centroids[1].x = 272.877f;  packet->centroids[1].y = 119.884f;
    packet->centroids[2].x = 300.900f;  packet->centroids[2].y = 185.904f;
    packet->centroids[3].x = 319.573f;  packet->centroids[3].y = 253.054f;
    packet->centroids[4].x = 667.001f;  packet->centroids[4].y = 456.964f;
    // 3. Calculate checksum
    uint8_t calc_checksum = 0;
    uint8_t *ptr = (uint8_t*)packet;

    // XOR everything except the last byte (the checksum field itself)
    for(int i = 0; i < sizeof(FPGA_Packet_t) - 1; i++) {
        calc_checksum ^= ptr[i];
    }
    packet->checksum = calc_checksum;
}
