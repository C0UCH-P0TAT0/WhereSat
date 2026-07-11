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
#include <stdint.h>

#define MAX_CENTROIDS 100
#define FPGA_PACKET_HEADER 0xAA

/**
 * @brief Centroid using 32-bit floats for subpixel precision.
 */
typedef struct {
    float x; 
    float y;
    float mass;  
} Centroid_t;

/**
 * @brief The unified Data Model for both SPI (FPGA) and UART (Host).
 * __attribute__((packed)) is critical here to ensure the SPI stream 
 * maps exactly to the struct members.
 */
typedef struct __attribute__((packed)) {
    uint8_t header;                   // Used by SPI
    uint8_t count;                    // Used by both
    Centroid_t centroids[MAX_CENTROIDS]; // 26 * 8 bytes = 208 bytes
    uint8_t checksum;                 // Used by SPI
} FPGA_Packet_t;

#endif