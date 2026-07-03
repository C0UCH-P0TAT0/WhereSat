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


    packet->centroids[0].x = 430.250f; 
    packet->centroids[0].y = 272.508f;

    packet->centroids[1].x = 293.709f;
    packet->centroids[1].y = 525.921f;

    packet->centroids[2].x = 1019.443f; 
    packet->centroids[2].y = 224.526f;

    packet->centroids[3].x = 611.826f;  
    packet->centroids[3].y = 574.466f;

    packet->centroids[4].x = 466.203f;  
    packet->centroids[4].y = 738.495f;

    // 3. Calculate checksum
    uint8_t calc_checksum = 0;
    uint8_t *ptr = (uint8_t*)packet;

    // XOR everything except the last byte (the checksum field itself)
    for(int i = 0; i < sizeof(FPGA_Packet_t) - 1; i++) {
        calc_checksum ^= ptr[i];
    }
    packet->checksum = calc_checksum;
}
