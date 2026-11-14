#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

/* =====================================================================
 * bsp_imu.h —— YbImu 姿态传感器驱动（BSP 层，UART）
 *
 * 文件作用：
 *   把串口收到的 YbImu 数据帧解析成结构体，给上层提供「读姿态」的接口。
 *   帧格式（厂家协议，小端）：
 *     [0x7E][0x23][len][func][data...][sum]
 *     len  = 整帧字节数（含校验），sum = 前面所有字节之和 & 0xFF
 *   模块上电后自动按 100 Hz 上报，不需要发请求。
 *
 * 命名对照：
 *   IMU      = Inertial Measurement Unit（惯性测量单元 / 姿态传感器）
 *   Init     = 初始化（挂接收中断）
 *   Task     = 周期任务（把环形缓冲区里的字节解析成数据）
 *   Euler    = 欧拉角 roll / pitch / yaw，单位 rad
 *   Quat     = 四元数 w / x / y / z
 *   Baro     = 气压计（高度 / 温度 / 气压 / 气压差）
 *   report   = 上报（模块向上位机发数据）
 *   algo     = 融合算法类型（6 = 六轴，9 = 九轴）
 *   Online   = 在线（最近一段时间收到过有效帧）
 * ===================================================================== */

#define BSP_IMU_OK             0U
#define BSP_IMU_ERROR          1U

/* 功能字（与厂家协议一致） */
#define BSP_IMU_FUNC_VERSION   0x01U   /* 固件版本 */
#define BSP_IMU_FUNC_RAW       0x04U   /* 原始加速度/角速度/磁场 */
#define BSP_IMU_FUNC_QUAT      0x16U   /* 四元数 */
#define BSP_IMU_FUNC_EULER     0x26U   /* 欧拉角（rad） */
#define BSP_IMU_FUNC_BARO      0x32U   /* 气压计 */
#define BSP_IMU_FUNC_RATE      0x60U   /* 设置上报频率 10~100 Hz */
#define BSP_IMU_FUNC_ALGO      0x61U   /* 设置融合算法 6 轴 / 9 轴 */
#define BSP_IMU_FUNC_REQUEST   0x80U   /* 主动请求数据 */
#define BSP_IMU_FUNC_STATE     0x81U   /* 命令执行结果 */

/* 传感器数据（单位已换算好，上位机直接用） */
typedef struct {
    float   accel[3];        /* 加速度 g，索引 0/1/2 = x/y/z */
    float   gyro[3];         /* 角速度 rad/s */
    float   mag[3];          /* 磁场 uT */
    float   quat[4];         /* 四元数 w/x/y/z */
    float   euler[3];        /* 欧拉角 rad，索引 0/1/2 = roll/pitch/yaw */
    float   baro[4];         /* 高度 m / 温度 ℃ / 气压 / 气压差 */
    uint8_t version[3];      /* 固件版本 H/M/L */
    uint32_t frame_count;    /* 已解析的有效帧数，持续增长说明链路正常 */
    uint32_t byte_count;     /* 串口收到的原始字节数（诊断用） */
    uint32_t last_tick;      /* 最近一帧的时间戳 HAL_GetTick() */
} BSP_IMU_Data_t;

/* 作用：挂上串口接收中断，开始收数据。
 * 参数 huart：传感器接的那个串口句柄（如 &huart2），波特率必须是 115200。
 * 返回 BSP_IMU_OK 表示中断已开，BSP_IMU_ERROR 表示失败 */
uint8_t BSP_IMU_Init(UART_HandleTypeDef *huart);

/* 作用：把缓冲区里已收到的字节解析成数据，非阻塞。
 * 放在主循环里周期调用（200 Hz 足够），解析结果用 BSP_IMU_GetData 取 */
void BSP_IMU_Task(void);

/* 作用：取数据指针。返回 NULL 表示还没 Init。
 * 注意：返回的是驱动内部变量，上层只读，不要改 */
const BSP_IMU_Data_t *BSP_IMU_GetData(void);

/* 作用：判断传感器是否在线（最近 timeout_ms 内收到过有效帧）。
 * 参数 timeout_ms：超时时间，建议 200~500 ms */
uint8_t BSP_IMU_IsOnline(uint32_t timeout_ms);

/* 作用：设置上报频率（会写入模块配置）。
 * 参数 hz：10~100，超出范围会被夹紧。返回 BSP_IMU_OK 成功 */
uint8_t BSP_IMU_SetReportRate(uint8_t hz);

/* 作用：设置融合算法（会写入模块配置）。
 * 参数 algo：6 = 六轴（不用磁力计），9 = 九轴（航向不漂）。返回 BSP_IMU_OK 成功 */
uint8_t BSP_IMU_SetAlgoType(uint8_t algo);

/* 作用：主动请求一帧数据（模块若默认静默，用这个触发上报）。
 * 参数 func：要请求的功能字，如 BSP_IMU_FUNC_EULER / BSP_IMU_FUNC_RAW。
 * 注意：请求命令要从 PE0（USART1_TX）发到模块的 RX 脚才有用 */
uint8_t BSP_IMU_RequestData(uint8_t func);

#endif
