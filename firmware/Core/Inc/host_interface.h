#ifndef INC_HOST_INTERFACE_H_
#define INC_HOST_INTERFACE_H_

#include "main.h"
#include "fpga_interface.h"
#include "camera_geometry.h" // <--- ADD THIS for Vector3_t

#define HOST_SOF 0x55

HAL_StatusTypeDef host_receive_packet(FPGA_Packet_t *packet, Vector3_t *gyro_out);

#endif
