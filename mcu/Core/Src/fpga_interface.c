#ifndef INC_FPGA_INTERFACE_H_
#define INC_FPGA_INTERFACE_H_

#include <stdint.h>

#define MAX_CENTROIDS 26
#define HOST_SOF      0x55

typedef struct {
    float x;
    float y;
    float mass; // Added for brightness sorting
} Centroid_t;

typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t count;
    Centroid_t centroids[MAX_CENTROIDS];
    uint8_t checksum;
} FPGA_Packet_t;

#endif