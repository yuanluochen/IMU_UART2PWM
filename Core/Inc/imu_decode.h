/**
 ******************************************************************************
 * @file    imu_decode.h
 * @brief   HiPNUC IMU 串口帧的拼帧与解码（极简版）
 ******************************************************************************
 * 帧格式（小端）：
 *   [0][1]   [2][3]      [4][5]   [6 ...]
 *   5A A5    长度(len)    CRC     数据区(len 字节)
 *
 * 数据区第 1 字节是型号：0x91 = HI91（浮点格式）
 ******************************************************************************
 */
#ifndef __IMU_DECODE_H__
#define __IMU_DECODE_H__

#include <stdint.h>

// IMU 数据
typedef struct __attribute__((packed)){
  float acc[3]; // 加速度，单位 m/s²
  float gyr[3]; // 角速度，单位 deg/s
  float rpy[3]; // 欧拉角，单位 deg +-180deg
} imu_data_t;
// H83数据
typedef struct __attribute__((packed)){
  uint8_t tag; // 数据标签 == 0x83
  uint16_t main_status;
  uint8_t status_ext;
  uint32_t data_bitmap; //数据位段 == 0x0B (0b 0000 1011)
  imu_data_t imu_data;
} H83_data_t;
//帧头
typedef struct __attribute__((packed))
{
  uint16_t sof; // 0x5AA5
  uint16_t len; // 数据域长度（小端，不含帧头、长度和 CRC）
  uint16_t crc; // 对帧头、长度、数据域的 16 位 CRC（不含 CRC 字段本身
} frame_head_t;
// 单帧数据格式
typedef struct __attribute__((packed)){
  frame_head_t frame_head;
  H83_data_t h83_data;
} frame_data_t;

#define FRAME_SOF1 0X5A
#define FRAME_SOF2 0XA5
#define FRAME_LEN (sizeof(H83_data_t))
#define H83_TAG 0x83
#define H83_bitmap 0x0B

/**
 * @brief 对缓冲区内数据进行解码
 * 
 * @param buf_data 接收缓冲区数据
 * @param pos 接收缓冲区当前起始数据位置
 * @param buf_len 缓冲区大小
 * @param out imu解码数据
 * 
 */
int imu_decode_byte(uint8_t* buf_data, uint16_t* pos, uint16_t buf_len, imu_data_t *out);
/**
 * @brief 获取累计成功接收的次数
 */
uint16_t get_receive_ok_count();
/**
 * @brief 累计接收成功次数归零
 */
void clear_receive_count();
#endif /* __IMU_DECODE_H__ */
