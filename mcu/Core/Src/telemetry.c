/**
 * @file telemetry.c
 * @brief Implementation of the expanded binary telemetry (Bias Removed).
 */

#include "telemetry.h"
#include "usart.h"

typedef struct __attribute__((packed)) {
    uint8_t sof;
    
    // 18 floats matching Python struct.unpack("<18fBB")
    float q_est_x, q_est_y, q_est_z, q_est_w;
    float w_est_x, w_est_y, w_est_z;
    float torque_x, torque_y, torque_z;
    float q_quest_x, q_quest_y, q_quest_z, q_quest_w;
    float gyro_meas_x, gyro_meas_y, gyro_meas_z;
    float innovation_dot;
    
    // 2 uint8s
    uint8_t locked;
    uint8_t count;
    
    // 1 uint16 CRC
    uint16_t crc;
} TelemetryPacket_t;

static uint16_t crc16_telem(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

HAL_StatusTypeDef telemetry_send(
    const Quaternion_t *q_est, 
    const Vector3_t *w_est, 
    const Vector3_t *torque, 
    const Quaternion_t *q_quest,
    const Vector3_t *gyro_meas,
    float innovation_dot,
    bool locked, 
    uint8_t num_stars
) {
    TelemetryPacket_t pkt;
    pkt.sof = TELEM_SOF;

    pkt.q_est_x = q_est->x; pkt.q_est_y = q_est->y; pkt.q_est_z = q_est->z; pkt.q_est_w = q_est->w;
    pkt.w_est_x = w_est->x; pkt.w_est_y = w_est->y; pkt.w_est_z = w_est->z;
    pkt.torque_x = torque->x; pkt.torque_y = torque->y; pkt.torque_z = torque->z;
    
    pkt.q_quest_x = q_quest->x; pkt.q_quest_y = q_quest->y; pkt.q_quest_z = q_quest->z; pkt.q_quest_w = q_quest->w;
    pkt.gyro_meas_x = gyro_meas->x; pkt.gyro_meas_y = gyro_meas->y; pkt.gyro_meas_z = gyro_meas->z;
    
    pkt.innovation_dot = innovation_dot;

    pkt.locked = locked ? 1 : 0;
    pkt.count = num_stars;

    pkt.crc = crc16_telem((uint8_t*)&pkt, sizeof(TelemetryPacket_t) - 2);

    return HAL_UART_Transmit(&huart2, (uint8_t*)&pkt, sizeof(TelemetryPacket_t), 100);
}