/*
 * error_detection.h
 *
 *  Created on: Jun 20, 2024
 *      Author: sajanduwal
 */

#ifndef INC_ERROR_DETECTION_H_
#define INC_ERROR_DETECTION_H_

#include "main.h"

uint16_t calculateCRC_CCITT_AX25(const uint8_t *data, size_t length);

uint16_t calc_CRC(const uint8_t *data, size_t length);

#endif /* INC_ERROR_DETECTION_H_ */
