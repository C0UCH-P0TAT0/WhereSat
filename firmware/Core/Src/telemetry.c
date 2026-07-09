/**
 * @file telemetry.c
 * @brief Implementation of the expanded binary telemetry.
 */

#include "telemetry.h"
#include "usart.h"

typedef struct __attribute__((packed)) {
    uint8_t sof;
    float qx, qy, qz, qw;
    float wx, wy, wz;
    float tx, ty, tz;
    uint8_t locked;
    uint8_t count;
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

HAL_StatusTypeDef telemetry_send(const Quaternion_t *q, const Vector3_t *omega, const Vector3_t *torque, bool locked, uint8_t num_stars) {
    TelemetryPacket_t pkt;
    pkt.sof = TELEM_SOF;

    pkt.qx = q->x; pkt.qy = q->y; pkt.qz = q->z; pkt.qw = q->w;
    pkt.wx = omega->x; pkt.wy = omega->y; pkt.wz = omega->z;
    pkt.tx = torque->x; pkt.ty = torque->y; pkt.tz = torque->z;

    pkt.locked = locked ? 1 : 0;
    pkt.count = num_stars;

    pkt.crc = crc16_telem((uint8_t*)&pkt, sizeof(TelemetryPacket_t) - 2);

    return HAL_UART_Transmit(&huart2, (uint8_t*)&pkt, sizeof(TelemetryPacket_t), 100);
}
