/**
 * @file host_interface.c
 * @brief Implementation of the HIL binary ingest.
 */

#include "host_interface.h"
#include "usart.h"
#include <string.h>

static uint16_t crc16(const uint8_t *data, uint16_t length, uint16_t start_crc) {
    uint16_t crc = start_crc;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

HAL_StatusTypeDef host_receive_packet(FPGA_Packet_t *packet, Vector3_t *gyro_out) {
    uint8_t header[2]; // [SOF, Count]
    uint16_t received_crc;
    uint8_t raw_centroids[MAX_CENTROIDS * 4];

    // 1. Sync: Wait for SOF
    while (1) {
        if (HAL_UART_Receive(&huart2, &header[0], 1, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
        if (header[0] == HOST_SOF) break;
    }

    // 2. Read Count
    if (HAL_UART_Receive(&huart2, &header[1], 1, 100) != HAL_OK) return HAL_ERROR;
    uint8_t count = header[1];
    if (count > MAX_CENTROIDS) return HAL_ERROR;

    // 3. Read Gyro Data (3 floats = 12 bytes)
    if (HAL_UART_Receive(&huart2, (uint8_t*)gyro_out, 12, 100) != HAL_OK) return HAL_ERROR;

    // 4. Read Centroids (count * 4 bytes)
    uint16_t payload_size = count * 4;
    if (HAL_UART_Receive(&huart2, raw_centroids, payload_size, 500) != HAL_OK) return HAL_ERROR;

    // 5. Read CRC
    if (HAL_UART_Receive(&huart2, (uint8_t*)&received_crc, 2, 100) != HAL_OK) return HAL_ERROR;

    // 6. CRC Verification (Header + Gyro + Centroids)
    uint16_t calc = crc16(header, 2, 0xFFFF);
    calc = crc16((uint8_t*)gyro_out, 12, calc);
    calc = crc16(raw_centroids, payload_size, calc);

    if (calc != received_crc) return HAL_ERROR;

    // 7. Populate Struct
    packet->count = count;
    for (int i = 0; i < count; i++) {
        uint16_t x_raw = raw_centroids[i*4]   | (raw_centroids[i*4+1] << 8);
        uint16_t y_raw = raw_centroids[i*4+2] | (raw_centroids[i*4+3] << 8);
        packet->centroids[i].x = (float)x_raw;
        packet->centroids[i].y = (float)y_raw;
    }

    return HAL_OK;
}
