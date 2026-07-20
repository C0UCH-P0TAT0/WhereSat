/**
 * @file mock_fpga.h
 * @brief Header for FPGA simulation/mocking.
 *
 * @author Aditya (WhereSat Team)
 */

#ifndef INC_MOCK_FPGA_H_
#define INC_MOCK_FPGA_H_

#include "fpga_interface.h"

/**
 * @brief Fills a packet with mock star data for testing.
 */
void load_test_centroids(FPGA_Packet_t *packet);

#endif /* INC_MOCK_FPGA_H_ */
