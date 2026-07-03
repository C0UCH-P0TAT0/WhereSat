/**
 * @file fpga_interface.c
 * @brief Implementation of FPGA SPI communication with robust validation.
 *
 * Handles SPI reception with memory clearing on failure and multi-stage
 * packet validation (header, star count, and XOR checksum).
 *
 * @author Aditya (WhereSat Team)
 */

#include "fpga_interface.h"
#include <stdio.h>
#include <string.h>

extern SPI_HandleTypeDef hspi1;

HAL_StatusTypeDef fpga_receive_centroids(FPGA_Packet_t *packet) {
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS Low
    status = HAL_SPI_Receive(&hspi1, (uint8_t *)packet, sizeof(FPGA_Packet_t), 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS High

    // Requirement: Clear packet if SPI fails
    if (status != HAL_OK) {
        memset(packet, 0, sizeof(FPGA_Packet_t));
    }

    return status;
}

uint8_t fpga_validate_packet(FPGA_Packet_t *packet) {
    // 1. Header Validation
    if (packet->header != FPGA_PACKET_HEADER) {
        printf("Validation Error: Bad Header (0x%02X)\r\n", packet->header);
        return 0;
    }

    // 2. Requirement: Count Validation
    if (packet->count > MAX_CENTROIDS) {
        printf("Validation Error: Count %d exceeds MAX %d\r\n", packet->count, MAX_CENTROIDS);
        return 0;
    }

    // 3. Checksum Validation
    uint8_t calc_checksum = 0;
    uint8_t *raw_data = (uint8_t *)packet;
    for (int i = 0; i < sizeof(FPGA_Packet_t) - 1; i++) {
        calc_checksum ^= raw_data[i];
    }

    // Requirement: Swap message (Expected = calculated, Received = packet)
    if (calc_checksum != packet->checksum) {
        printf("Validation Error: Checksum Mismatch (Expected 0x%02X, Received 0x%02X)\r\n",
                calc_checksum, packet->checksum);
        return 0;
    }

    return 1;
}
