/**
 * @file fpga_interface.h
 * @brief Definitions and structures for SPI communication with the FPGA.
 *
 * This file defines the centroid data structures and the protocol for
 * receiving star coordinates from the FPGA image processing pipeline.
 *
 * @author Aditya (WhereSat Team)
 */

#ifndef INC_FPGA_INTERFACE_H_
#define INC_FPGA_INTERFACE_H_

#include "main.h"

#define MAX_CENTROIDS 26
#define FPGA_PACKET_HEADER 0xAA

/**
 * @brief Structure representing a single star centroid in pixel coordinates.
 */
typedef struct {
    float x;
    float y;
} Centroid_t;

/**
 * @brief Structure for the full SPI data packet from the FPGA.
 */
typedef struct {
    uint8_t header;
    uint8_t count;
    Centroid_t centroids[MAX_CENTROIDS];
    uint8_t checksum;
} FPGA_Packet_t;

/* Function Prototypes */
HAL_StatusTypeDef fpga_receive_centroids(FPGA_Packet_t *packet);
uint8_t fpga_validate_packet(FPGA_Packet_t *packet);
void fpga_print_centroids(FPGA_Packet_t *packet);

#endif /* INC_FPGA_INTERFACE_H_ */
