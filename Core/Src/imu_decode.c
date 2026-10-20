/**
 ******************************************************************************
 * @file    imu_decode.c
 * @brief   HiPNUC IMU 串口帧的拼帧与解码（极简版）
 ******************************************************************************
 */
#include "imu_decode.h"
#include "crc.h"

#define HEADER_SIZE 6   /* 帧头：同步2 + 长度2 + CRC2 */
#define MAX_PAYLOAD 506 /* 协议：一帧数据最多 506 字节 */
#define FRAME_MAX (HEADER_SIZE + MAX_PAYLOAD)

/* ---- 拼帧状态 ---- */
static uint8_t s_frame[FRAME_MAX]; /* 攒一帧的地方 */
static uint16_t s_n = 0;           /* 已攒字节数 */
static uint16_t s_payload_len = 0; /* 本帧数据长度 */
static uint16_t s_frame_len = 0;   /* 本帧总长度 = 6 + len */
// 累计成功/失败接收此时
uint16_t rx_ok_count = 0;

/* 从 4 个字节读一个小端 float */
static float get_f32(const uint8_t *p)
{
  union
  {
    uint32_t u;
    float f;
  } v;
  v.u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  return v.f;
}
/**
 * memcpy 针对缓冲区
 */
static void memcpy_buf(uint8_t * src, uint16_t src_begin, uint16_t src_len, uint8_t * dest, uint16_t cpy_len){
  uint16_t cur_pos = src_begin;
  for (int i = 0; i < cpy_len; i++)
  {
    dest[i] = src[cur_pos];
    cur_pos = (cur_pos + 1) % src_len;
  }
}

/**
 * @brief 寻找起始位 0X5A 0XA5
 */
static int16_t find_SOF(uint8_t *buf_data, uint16_t *pos, uint16_t buf_len){
  uint16_t cur_pos = *pos;
  uint16_t next_pos = cur_pos + 1;
  uint8_t count = 0; // 遇到一次起始位置记一次数，第一次是开始，第二次说明已经遍历一遍了，直接结束
  if (next_pos >= buf_len){
    next_pos = 0;
  }
  while (!(buf_data[cur_pos] == FRAME_SOF1 && buf_data[next_pos] == FRAME_SOF2)){
    if (cur_pos == *pos){
      count++;
      if (count == 2){
        //说明已经遇到两次起始位置，已经循环一遍，退出循环
        break;
      }
    }
    cur_pos = next_pos;
    next_pos++;
    if (next_pos >= buf_len)
    {
      next_pos = 0;
    }
  }
  if (count != 2){
    //说明找到起始位置了
    return cur_pos;
  }else{

    return -1;
  }
}

int imu_decode_byte(uint8_t *buf_data, uint16_t *pos, uint16_t buf_len, imu_data_t *out){
  // 寻找起始位
  int16_t sof_pos = find_SOF(buf_data, pos, buf_len);
  if (sof_pos == -1){
    //说明没找到那个数字，直接终止这个函数,同时POS不变
    return -1;
  }
  //找到了SOF，向后取帧头位判断整个数据状态
  uint8_t head_data[sizeof(frame_head_t)] = {};
  //拷贝数据
  memcpy_buf(buf_data, sof_pos, buf_len, head_data, sizeof(frame_head_t));
  //数据转换
  frame_head_t * frame_head = (frame_head_t*)head_data;
  //长度校验
  if (frame_head->len != sizeof(H83_data_t)){
    //长度不合理，直接跳过
    *pos = (sof_pos + 1) % buf_len;
    return -1;
  }
  //CRC
  if (!frame_crc_ok(buf_data + sof_pos, frame_head->len)){
    //CRC 没通过直接跳过，到CRC位置
    *pos = (sof_pos + 1) % buf_len;
    return -1;
  }
  rx_ok_count++;
  //CRC通过，解码
  uint8_t frame_buf[sizeof(frame_data_t)] = {};
  // 拷贝数据
  memcpy_buf(buf_data, sof_pos, buf_len, frame_buf, sizeof(frame_data_t));
  //数据转换
  frame_data_t *frame_data = (frame_data_t *)frame_buf;
  *out = frame_data->h83_data.imu_data;
  *pos = (sof_pos + sizeof(frame_head_t) + frame_head->len) % buf_len;

  return 0;
}

uint16_t get_receive_ok_count(){
  return rx_ok_count;
}
void clear_receive_count(){
  rx_ok_count = 0;
}