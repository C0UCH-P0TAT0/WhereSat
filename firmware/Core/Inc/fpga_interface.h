/**
 * @file fpga_interface.h
 * @brief Definitions and packed structures for SPI communication with the FPGA.
 *
 * This file defines the centroid data structures. The struct is packed to
 * prevent compiler padding, ensuring 1:1 mapping with the FPGA SPI stream.
 *
 * @author Aditya (WhereSat Team)
 */

#ifndef INC_FPGA_INTERFACE_H_
#define INC_FPGA_INTERFACE_H_

#include "main.h"

#define MAX_CENTROIDS 26
#define FPGA_PACKET_HEADER 0xAA

typedef struct __attribute__((packed)) {
    float x;
    float y;
} Centroid_t;

typedef struct __attribute__((packed)) {
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
