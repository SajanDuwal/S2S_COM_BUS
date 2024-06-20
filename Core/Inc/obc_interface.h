/*
 * obc_interface.h
 *
 *  Created on: Jun 19, 2024
 *      Author: sajanduwal
 */

#ifndef INC_OBC_INTERFACE_H_
#define INC_OBC_INTERFACE_H_

#include "main.h"
#include "com_debug.h"

extern UART_HandleTypeDef huart2;

void WAIT_FOR_HANDSHAKE();

#endif /* INC_OBC_INTERFACE_H_ */
