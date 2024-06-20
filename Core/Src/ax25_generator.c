/*
 * ax25_generator.c
 *
 *  Created on: Jun 20, 2024
 *      Author: sajanduwal
 */

#include "ax25_generator.h"

#define INFO_LEN		(80)

uint8_t info_packet[INFO_LEN];
extern uint8_t tx_cmd[102];

void updatePacket(uint8_t *OBC_Rx_buffer) {
	int j = 0;
	int k = 2;
	int len_of_payload = OBC_Rx_buffer[1];
	myDebug("\nlen_of_payload: %d\r\n", len_of_payload);
	for (int i = 0; i < len_of_payload; i++) {
		info_packet[j] = OBC_Rx_buffer[k];
		j++;
		k++;
	}
	myDebug("info_packet: 0x%x\r\n", info_packet);
	for (int i = 0; i < sizeof(info_packet); i++) {
		myDebug(" %x", info_packet[i]);
	}
	myDebug("\r\n");
}

int AX_25PacketFormation(uint8_t *OBC_Rx_buffer) {

	updatePacket(OBC_Rx_buffer);
	uint8_t buf_packet[96];
	uint8_t buff_head[17];
	// AX.25 Packet header
	tx_cmd[0] = 0x73;

	// destination callsign
	tx_cmd[1] = 0x40;    // callsign  space 0x20 -> 1bit leftshift 0x40
	tx_cmd[2] = 0x40;	//callsign	N
	tx_cmd[3] = 0x40;	//callsign	2
	tx_cmd[4] = 0x40;	//callsign	S
	tx_cmd[5] = 0x40;	//callsign	I
	tx_cmd[6] = 0x40;	// callsign space
	tx_cmd[7] = 0xE0;	// destination SSID

	// source callsign					39 4E 32 53 49
	tx_cmd[8] = 0x72;    // callsign 9
	tx_cmd[9] = 0x9C;	//callsign	N
	tx_cmd[10] = 0x64;	//callsign	2
	tx_cmd[11] = 0xA6;	//callsign	S
	tx_cmd[12] = 0x92;	//callsign	I
	tx_cmd[13] = 0x40;	// callsign space
	tx_cmd[14] = 0x36;	// source SSID

	// control field
	tx_cmd[15] = 0x03;

	// PID control bit
	tx_cmd[16] = 0xF0;

	for (int a = 0; a < 17; a++) {
		buff_head[a] = tx_cmd[a];
	}

	uint16_t crc = 0;
	crc = calculateCRC_CCITT_AX25(buff_head, sizeof(buff_head));

	tx_cmd[17] = (crc >> 8) && 0xFF;
	tx_cmd[18] = crc & 0xFF;

	// information field
	int i = 19;
	for (int k = 0; k < sizeof(info_packet); k++) {
		tx_cmd[i] = info_packet[k];
		i++;
	}

	int j = 0;
	for (int k = 19; k < 99; k++) {
		buf_packet[j] = tx_cmd[k];
		j++;
	}
	// Calculate CRC-CCITT for the packet data starting from packet[1]
	crc = 0;
	crc = calculateCRC_CCITT_AX25(buf_packet, j-1);

	// Store CRC result in the packet array (from packet[1] to end of for loop)
	tx_cmd[i] = (crc >> 8) & 0xFF; // Most significant byte
	i++;
	tx_cmd[i] = crc & 0xFF;        // Least significant byte
	i++;
	// AX.25 Packet footer
	tx_cmd[i] = 0x73;
	myDebug("\npacket_len: %d\r\n", i + 1);
	myDebug("packet: 0x%x\r\n", tx_cmd);
	for (int j = 0; j <= i; j++) {
		myDebug(" %x", tx_cmd[j]);
	}
	myDebug("\r\n");
	delay_us(100);
	return i + 1;
}

