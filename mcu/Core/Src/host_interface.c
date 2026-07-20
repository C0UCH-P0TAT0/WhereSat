/**
 * @file host_interface.c
 * @brief HIL Ingest with Subpixel Floats, Mass, and SWV Tracing.
 */

#include "host_interface.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

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
    // Buffer: 2(hdr) + 12(gyro) + 26*12(payload) + 2(crc) = 328 bytes
    static uint8_t full_packet[512]; 
    uint8_t *ptr = full_packet;
    HAL_StatusTypeDef st;

    // 1. Sync SOF
    while (1) {
        st = HAL_UART_Receive(&huart2, ptr, 1, HAL_MAX_DELAY);
        if (st != HAL_OK) return HAL_ERROR;
        if (*ptr == HOST_SOF) break;
    }
    ptr++; 

    // 2. Read Count
    st = HAL_UART_Receive(&huart2, ptr, 1, 100);
    if (st != HAL_OK) return HAL_ERROR;
    uint8_t count = *ptr;
    ptr++;

    // 3. Read Gyro (12 bytes)
    st = HAL_UART_Receive(&huart2, ptr, 12, 100);
    if (st != HAL_OK) return HAL_ERROR;
    memcpy(gyro_out, ptr, 12);
    ptr += 12;

    // 4. Read Centroids (count * 12 bytes: X, Y, Mass)
    uint16_t payload_size = count * 12;
    if (payload_size > 0) {
        st = HAL_UART_Receive(&huart2, ptr, payload_size, 500);
        if (st != HAL_OK) return HAL_ERROR;
        ptr += payload_size;
    }

    // 5. Read CRC
    st = HAL_UART_Receive(&huart2, ptr, 2, 100);
    if (st != HAL_OK) return HAL_ERROR;
    uint16_t received_crc = ptr[0] | (ptr[1] << 8);
    ptr += 2;

    uint16_t total_len = ptr - full_packet;

    // --- SWV PACKET INSPECTOR ---
    printf("\r\n[RX] PACKET RECEIVED - %d bytes\r\n", total_len);
    printf("  Header: %02X %02X | Gyro: %.3f %.3f %.3f\r\n", 
           full_packet[0], full_packet[1], gyro_out->x, gyro_out->y, gyro_out->z);
    
    // 6. Verify CRC
    uint16_t calc = crc16(full_packet, total_len - 2, 0xFFFF);
    if (calc != received_crc) {
        printf("  [ERROR] CRC MISMATCH! (Calc: %04X, Recv: %04X)\r\n", calc, received_crc);
        return HAL_ERROR;
    }

    // 7. Unpack
    packet->count = count;
    uint8_t *payload_ptr = &full_packet[14];
    for (int i = 0; i < count; i++) {
        memcpy(&packet->centroids[i].x,    &payload_ptr[i*12],     4);
        memcpy(&packet->centroids[i].y,    &payload_ptr[i*12 + 4], 4);
        memcpy(&packet->centroids[i].mass, &payload_ptr[i*12 + 8], 4);
    }

    printf("  [RESULT] CRC OK. Packet Unpacked.\r\n");
    return HAL_OK;
}