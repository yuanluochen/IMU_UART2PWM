#include "crc.h"

#include <stdint.h>
#include <stddef.h>


uint16_t crc16(uint16_t crc, const uint8_t *buf, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)buf[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}


int frame_crc_ok(const uint8_t *frame, uint16_t payload_len)
{
  uint16_t crc = crc16(0, frame, 4);        /* SOF + LEN */
  crc = crc16(crc, frame + 6, payload_len); /* payload, skipping the CRC field */
  return crc == (uint16_t)(frame[4] | (frame[5] << 8));
}
