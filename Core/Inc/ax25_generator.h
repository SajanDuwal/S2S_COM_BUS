/*
 * ax25_generator.h
 *
 *  Created on: Jun 20, 2024
 *      Author: sajanduwal
 */

#ifndef INC_AX25_GENERATOR_H_
#define INC_AX25_GENERATOR_H_


#include "main.h"
#include "error_detection.h"
#include "com_debug.h"

int AX_25PacketFormation(uint8_t *OBC_Rx_buffer);

#endif /* INC_AX25_GENERATOR_H_ */
