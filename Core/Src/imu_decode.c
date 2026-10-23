/**
 ******************************************************************************
 * @file    imu_decode.c
 * @brief   HiPNUC IMU 串口帧的拼帧与解码
 ******************************************************************************
 */
#include "imu_decode.h"
#include "crc.h"

#define HEADER_SIZE 6   /* 帧头：同步2 + 长度2 + CRC2 */
#define MAX_PAYLOAD 506 /* 协议：一帧数据最多 506 字节 */
#define FRAME_MAX (HEADER_SIZE + MAX_PAYLOAD)

static float limit_amp(float value, float max){
  if (value > max)
    return max;
  else if (value < -max)
    return -max;
  else
    return value;
}

/* 把环形缓冲里的一帧“拉直”到这里，后面__attribute是为了强制对其到四字节*/
static uint8_t s_frame[FRAME_MAX] __attribute__((aligned(4)));

uint16_t rx_ok_count = 0;    /* 累计成功解码帧数 */
imu_rx_stat_t g_imu_rx_stat; /* 最近一次轮询的接收窗口统计 */

/* 回卷安全读取：从 src[begin] 起取 len 字节到 dest */
static void ring_read(const uint8_t *src, uint16_t src_len,
                      uint16_t begin, uint8_t *dest, uint16_t len)
{
  uint16_t p = begin;
  uint16_t i;
  for (i = 0; i < len; i++)
  {
    dest[i] = src[p];
    if (++p >= src_len)
      p = 0;
  }
}

/* 在窗口 [begin, begin+n) 内找 5A A5；返回相对偏移，找不到返回 -1 */
static int32_t find_sof_rel(const uint8_t *buf, uint16_t buf_len,
                            uint16_t begin, uint16_t n)
{
  uint16_t i;
  if (n < 2)
    return -1;
  for (i = 0; i + 1 < n; i++)
  {
    uint16_t a = (uint16_t)((begin + i) % buf_len);
    uint16_t b = (uint16_t)((a + 1 == buf_len) ? 0 : (a + 1));
    if (buf[a] == FRAME_SOF1 && buf[b] == FRAME_SOF2)
      return (int32_t)i;
  }
  return -1;
}

uint16_t imu_decode_poll(const uint8_t *buf, uint16_t buf_len,
                         uint16_t *rpos, uint16_t wpos, imu_data_t *out)
{
  uint16_t r = (uint16_t)(*rpos % buf_len);
  uint16_t w = (uint16_t)(wpos % buf_len);
  uint16_t avail;
  uint16_t frames = 0;
  uint16_t dropped = 0;
  uint16_t consumed = 0;

  /* 本次窗口：上次解码结束位置 -> DMA 当前写指针 */
  avail = (r <= w) ? (uint16_t)(w - r) : (uint16_t)(buf_len - r + w);

  g_imu_rx_stat.new_bytes = avail;
  g_imu_rx_stat.window_begin = r;
  g_imu_rx_stat.window_end = w;

  while (avail >= 2)
  {
    int32_t off = find_sof_rel(buf, buf_len, r, avail);
    uint16_t payload_len;
    uint16_t total;

    if (off < 0)
    {
      /* 整个窗口都没有帧头：只保留最后 1 字节，防止 5A 被切到下一次 */
      uint16_t skip = (uint16_t)(avail - 1);
      dropped += skip;
      consumed += skip;
      r = (uint16_t)((r + skip) % buf_len);
      avail = 1;
      break;
    }

    if (off > 0)
    {
      /* 帧头前面的垃圾丢掉 */
      dropped += (uint16_t)off;
      consumed += (uint16_t)off;
      r = (uint16_t)((r + off) % buf_len);
      avail = (uint16_t)(avail - off);
    }

    if (avail < HEADER_SIZE)
      break; /* 帧头还没收全，留到下一次 */

    ring_read(buf, buf_len, r, s_frame, HEADER_SIZE);

    payload_len = (uint16_t)(s_frame[2] | ((uint16_t)s_frame[3] << 8));

    if (payload_len == 0 || payload_len > MAX_PAYLOAD)
    {
      /* 长度非法：假帧头，丢 1 字节重新同步 */
      dropped++;
      consumed++;
      r = (uint16_t)((r + 1) % buf_len);
      avail--;
      continue;
    }

    total = (uint16_t)(HEADER_SIZE + payload_len);

    if (total > avail)
      break; /* 半帧，保留在缓冲里，等下次轮询接着拼 */

    /* 整帧到齐，先拉直成连续内存再算 CRC（解决跨缓冲区边界的问题） */
    ring_read(buf, buf_len, r, s_frame, total);

    if (!frame_crc_ok(s_frame, payload_len))
    {
      /* CRC 不过：假帧头，丢 1 字节重新同步 */
      dropped++;
      consumed++;
      r = (uint16_t)((r + 1) % buf_len);
      avail--;
      continue;
    }

    /* 只接受 H83 数据帧 */
    if (payload_len == sizeof(H83_data_t))
    {
      const frame_data_t *fd = (const frame_data_t *)s_frame;
      if (fd->h83_data.tag == H83_TAG && fd->h83_data.data_bitmap == H83_bitmap)
      {
        *out = fd->h83_data.imu_data;
        for (int i = 0; i < 3; i++){
          out->acc[i] = limit_amp(out->acc[i], MAX_ACC);
          out->gyr[i] = limit_amp(out->gyr[i], MAX_GYR);
          out->rpy[i] = limit_amp(out->rpy[i], MAX_RPY);
        }
        frames++;
        rx_ok_count++;
        g_imu_rx_stat.last_frame_start = r;
        g_imu_rx_stat.last_frame_len = total;
      }
    }

    consumed += total;
    r = (uint16_t)((r + total) % buf_len);
    avail = (uint16_t)(avail - total);
  }

  *rpos = r;
  g_imu_rx_stat.consumed = consumed;
  g_imu_rx_stat.dropped = dropped;
  g_imu_rx_stat.frames = frames;
  g_imu_rx_stat.total_bytes += g_imu_rx_stat.new_bytes;

  return frames;
}

uint16_t get_receive_ok_count(void)
{
  return rx_ok_count;
}

void clear_receive_count(void)
{
  rx_ok_count = 0;
}