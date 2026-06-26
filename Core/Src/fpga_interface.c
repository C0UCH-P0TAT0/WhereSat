/**
 * @file fpga_interface.c
 * @brief Implementation of FPGA SPI communication logic.
 *
 * Handles the low-level SPI transactions, including manual CS control
 * and basic packet validation to ensure data integrity from the FPGA.
 *
 * @author Aditya (WhereSat Team)
 */

#include "fpga_interface.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

/**
 * @brief Pulls CS low, receives the packet, and releases CS.
 * @param packet Pointer to the FPGA_Packet_t structure to fill.
 * @return HAL status of the SPI transaction.
 */

HAL_StatusTypeDef fpga_receive_centroids(FPGA_Packet_t *packet) {
    HAL_StatusTypeDef status;

    // Pull CS Low (Active)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // Receive the packet (Size of header + count + centroids + checksum)
    // We use a timeout of 100ms
    status = HAL_SPI_Receive(&hspi1, (uint8_t *)packet, sizeof(FPGA_Packet_t), 100);

    // Pull CS High (Inactive)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    return status;
}

/**
 * @brief Validates the packet header and checksum.
 * @param packet Pointer to the received packet.
 * @return 1 if valid, 0 if corrupt.
 */
uint8_t fpga_validate_packet(FPGA_Packet_t *packet) {
    if (packet->header != FPGA_PACKET_HEADER) {
        printf("Validation Error: Bad Header (0x%02X)\r\n", packet->header);
        return 0;
    }

    uint8_t calc_checksum = 0;
    uint8_t *raw_data = (uint8_t *)packet;
    for (int i = 0; i < sizeof(FPGA_Packet_t) - 1; i++) {
        calc_checksum ^= raw_data[i];
    }

    if (calc_checksum != packet->checksum) {
        printf("Validation Error: Checksum Mismatch (Expected 0x%02X, Got 0x%02X)\r\n",
                packet->checksum, calc_checksum);
        return 0;
    }

    return 1; // Success
}

/**
 * @brief Debug function to print received centroids to UART.
 */
void fpga_print_centroids(FPGA_Packet_t *packet) {
    printf("FPGA Packet: %d stars detected\r\n", packet->count);
    for (int i = 0; i < packet->count && i < MAX_CENTROIDS; i++) {
        printf("  Star %d: X=%.2f, Y=%.2f\r\n", i, packet->centroids[i].x, packet->centroids[i].y);
    }
}
