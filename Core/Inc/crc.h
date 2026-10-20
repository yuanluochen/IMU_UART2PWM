#ifndef CRC_H
#define CRC_H
#include "main.h"

uint16_t crc16(uint16_t crc, const uint8_t *buf, size_t len);

int frame_crc_ok(const uint8_t *frame, uint16_t payload_len);

#endif
