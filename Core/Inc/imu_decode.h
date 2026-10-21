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
typedef struct __attribute__((packed))
{
  float acc[3]; // 加速度，单位 m/s²
  float gyr[3]; // 角速度，单位 deg/s
  float rpy[3]; // 欧拉角，单位 deg +-180deg
} imu_data_t;

// H83数据
typedef struct __attribute__((packed))
{
  uint8_t tag; // 数据标签 == 0x83
  uint16_t main_status;
  uint8_t status_ext;
  uint32_t data_bitmap; // 数据位段 == 0x0B
  imu_data_t imu_data;
} H83_data_t;

// 帧头
typedef struct __attribute__((packed))
{
  uint16_t sof; // 0x5AA5
  uint16_t len; // 数据域长度（小端）
  uint16_t crc; // CRC16
} frame_head_t;

// 单帧数据格式
typedef struct __attribute__((packed))
{
  frame_head_t frame_head;
  H83_data_t h83_data;
} frame_data_t;

#define FRAME_SOF1 0X5A
#define FRAME_SOF2 0XA5
#define FRAME_LEN (sizeof(H83_data_t))
#define H83_TAG 0x83
#define H83_bitmap 0x0B

/* ★ 新增：接收窗口统计，用来观察“这次到底收了哪些字节、从哪到哪” */
typedef struct
{
  uint16_t new_bytes;        /* 本次新增字节数 = 窗口长度 */
  uint16_t window_begin;     /* 窗口起点（环形缓冲下标） */
  uint16_t window_end;       /* 窗口终点 = DMA 当前写指针 */
  uint16_t consumed;         /* 本次取走的字节数（有效帧 + 丢弃） */
  uint16_t dropped;          /* 本次丢弃的字节数（无帧头/长度错/CRC错） */
  uint16_t frames;           /* 本次解出的帧数 */
  uint16_t last_frame_start; /* 最近一帧的起始下标 */
  uint16_t last_frame_len;   /* 最近一帧的总字节数（6 + len） */
  uint32_t overrun;          /* 环形缓冲被覆盖的次数 */
  uint32_t total_bytes;      /* 上电以来累计收到的字节数 */
} imu_rx_stat_t;

extern imu_rx_stat_t g_imu_rx_stat;

/* ★ 新增：DMA 剩余计数 -> 环形缓冲写指针 */
static inline uint16_t imu_dma_write_pos(uint16_t dma_counter, uint16_t buf_len)
{
  return (dma_counter >= buf_len) ? 0 : (uint16_t)(buf_len - dma_counter);
}

/**
 * @brief 只解码 [上次解码结束位置, 本次 DMA 写指针) 之间的新数据
 * @param buf     接收缓冲区（DMA 循环写入）
 * @param buf_len 缓冲区大小
 * @param rpos    读写：上次解码结束位置，返回后更新为本次解码结束位置
 * @param wpos    本次 DMA 写入位置
 * @param out     解出的最新一帧 IMU 数据
 * @return 本次成功解出的帧数
 */
uint16_t imu_decode_poll(const uint8_t *buf, uint16_t buf_len,
                         uint16_t *rpos, uint16_t wpos, imu_data_t *out);

uint16_t get_receive_ok_count(void);
void clear_receive_count(void);

#endif /* __IMU_DECODE_H__ */