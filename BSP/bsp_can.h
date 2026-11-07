#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include <stdint.h>

/* =====================================================================
 * bsp_can.h —— CAN 收发驱动（BSP 层）
 *
 * 文件作用：
 *   把 CubeMX 生成的 FDCAN1 再包一层，给上层提供三个动作：
 *   初始化、发一帧、收一帧。上层只认 ID + 8 字节，不用碰 HAL 句柄。
 *
 * 命名对照：
 *   BSP      = Board Support Package（板级支持包）
 *   CAN      = Controller Area Network（控制器局域网）
 *   Init / Send / Recv = 初始化 / 发送 / 接收
 *   std_id   = Standard ID（11 位标准帧 ID）
 *   OK / ERROR = 0 成功 / 1 失败
 * ===================================================================== */

#define BSP_CAN_OK    0U
#define BSP_CAN_ERROR 1U

/* 作用：初始化 FDCAN1 的全局过滤并启动。CubeMX 不生成这两步，必须自己做。
 * 返回 BSP_CAN_OK 表示可以收发了 */
uint8_t BSP_CAN_Init(void);

/* 作用：发一帧标准帧，固定 8 字节。
 * 只支持经典 CAN / 不切波特率，因为 DM 电机和 M2006 都只认这个格式 */
uint8_t BSP_CAN_Send(uint32_t std_id, const uint8_t data[8]);

/* 作用：轮询取一帧，不会阻塞。
 * 返回 BSP_CAN_OK 表示 data 里是有效数据；BSP_CAN_ERROR 表示 FIFO 是空的 */
uint8_t BSP_CAN_Recv(uint32_t *std_id, uint8_t data[8]);

#endif
