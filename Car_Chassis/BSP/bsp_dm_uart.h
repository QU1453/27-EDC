/* =====================================================================
 * bsp_dm_uart.h —— 达妙 DM3520-1EC 驱动器「调试串口」驱动
 *
 * 手册结论（说明书 V1.0 第 4、5 页）：
 *   - 驱动器 3Pin 串口是配置口，UART 921600 8N1，用于读参数/改 ID/校准/升级；
 *   - 电机运行控制只能走 CAN（1 Mbps），串口不能直接跑电机。
 *
 * 硬件对应（CubeMX 已配好，见 usart.c）：
 *   USART3：PB10 = TX，PB11 = RX   → 1 号驱动器调试口
 *   UART4 ：PC10 = TX，PC11 = RX   → 2 号驱动器调试口
 *
 * 用法：
 *   BSP_DM_UART_Init();                       // 上电后调一次，改成 921600
 *   BSP_DM_UART_Send(port, buf, len);         // 发原始帧（参数读写协议按手册拼）
 *   BSP_DM_UART_GetLine(port, text, 40);      // 取驱动器打印的最近一行 ASCII
 * ===================================================================== */
#ifndef __BSP_DM_UART_H
#define __BSP_DM_UART_H

#include "main.h"

#define BSP_DM_UART_NUM        2U
#define BSP_DM_UART_PORT_3     0U      /* USART3：PB10 TX / PB11 RX */
#define BSP_DM_UART_PORT_4     1U      /* UART4 ：PC10 TX / PC11 RX */
#define BSP_DM_UART_BAUD       921600U /* 达妙配置口固定 921600 8N1 */

/* 作用：把 USART3/UART4 从 115200 改成 921600 8N1（不改 CubeMX 生成文件），并挂接收中断 */
void BSP_DM_UART_Init(void);

/* 作用：环形缓冲里还有多少字节可读 */
uint16_t BSP_DM_UART_Available(uint8_t port);

/* 作用：从环形缓冲取数据，返回实际字节数 */
uint16_t BSP_DM_UART_Read(uint8_t port, uint8_t *buf, uint16_t max);

/* 作用：累计收到的字节数（判断线路通不通用） */
uint32_t BSP_DM_UART_RxCount(uint8_t port);

/* 作用：取最近一行以 \n 结尾的可打印 ASCII（驱动器上电横幅用），返回长度 */
uint16_t BSP_DM_UART_GetLine(uint8_t port, char *out, uint16_t max);

/* 作用：最近一行收全的时刻（HAL_GetTick），0 = 还没收到过整行 */
uint32_t BSP_DM_UART_LineTick(uint8_t port);

/* 作用：发送原始字节，返回 0 = 成功 */
uint8_t BSP_DM_UART_Send(uint8_t port, const uint8_t *buf, uint16_t len);

/* 作用：给 HAL 回调分发用，返回 1 表示这个串口属于本模块（已在内部处理） */
uint8_t BSP_DM_UART_OnRx(UART_HandleTypeDef *huart);

/* 作用：串口出错后重挂接收 */
uint8_t BSP_DM_UART_OnError(UART_HandleTypeDef *huart);

#endif /* __BSP_DM_UART_H */
