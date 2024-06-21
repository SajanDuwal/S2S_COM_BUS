/*
 * error_detection.c
 *
 *  Created on: Jun 20, 2024
 *      Author: sajanduwal
 */

#include "error_detection.h"

// Function to calculate CRC-CCITT for AX.25 frames
uint16_t calculateCRC_CCITT_AX25(const uint8_t *data, size_t length) {
	uint16_t crc = 0xFFFF; // Initialize CRC register with 0xFFFF
	uint16_t CRC_POLY = 0x1021; // CRC polynomial for CCITT (0x1021)

	// Iterate through each byte of the input data
	for (size_t i = 0; i < length; i++) {
		crc ^= ((uint16_t) data[i] << 8); // XOR CRC with next byte of input data

		// Iterate through each bit of the current byte
		for (int j = 0; j < 8; j++) {
			if (crc & 0x8000) { // If MSB of CRC is 1
				crc = (crc << 1) ^ CRC_POLY; // Left shift CRC and XOR with polynomial
			} else {
				crc <<= 1; // Left shift CRC
			}
		}
	}

	return crc; // Return calculated CRC
}


uint16_t calc_CRC(const uint8_t *data, size_t length) {

	uint16_t crcReg = 0xFFFF;	// Initialize the CRC register with 0xFFFF
	uint16_t calc = 0x8408;		// Polynomial for CRC-16
	uint16_t w;
	int i, j;
	uint8_t calc_data[length];  // in length bytes, actual_size are data bytes for CRC

	// Copy data into calc_data
	for (i = 0; i < length; i++) {
		calc_data[i] = data[i];
		// Iterate over each byte of data
		for (j = 0; j < 8; j++) {
			w = (crcReg ^ calc_data[i]) & 0x0001; // XOR the LSB of crcReg with the LSB of calc_data
			crcReg = crcReg >> 1;			// Right-shift the crcReg by 1 bit
			if (w == 1) {
				crcReg = crcReg ^ calc;	// If w is 1, XOR the crcReg with the polynomial
			}
			calc_data[i] = calc_data[i] >> 1;// Right-shift the data byte by 1 bit
		}
	}
	crcReg = crcReg ^ 0xFFFF;						// Final XOR with 0xFFFF
	return crcReg;
}


