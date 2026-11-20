#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

/* =====================================================================
 * bsp_imu.h —— YbImu 姿态传感器驱动（BSP 层，I2C）
 *
 * 硬件接线（STM32G473VET6，I2C1）：
 *   PB7 = I2C1_SDA  → 传感器 SDA
 *   PA15 = I2C1_SCL → 传感器 SCL（原 PB8，PB8=BOOT0 不能带上拉）
 *   传感器 3V3 / GND 接板子 3V3 / GND（模块丝印 3V3、GND、SDA、SCL）
 *
 * 通信方式（厂家 I2C 寄存器表，7 位地址 0x23）：
 *   读：起始 + 地址(写) + 寄存器号 + 重启 + 地址(读) + N 字节
 *       对应 HAL_I2C_Mem_Read(..., MEMADD_SIZE_8BIT, ...)
 *   写：起始 + 地址(写) + 寄存器号 + 1 字节数据
 *   多字节数据小端（低字节在前）；寄存器地址自增，可连续读。
 *
 * 寄存器映射：
 *   0x01~0x03  VERSION_H/M/L        uint8 ×3
 *   0x04       ACCEL_X_L            int16 ×3（X/Y/Z）
 *   0x0A       GYRO_X_L             int16 ×3
 *   0x10       MAG_X_L              int16 ×3
 *   0x16       QUAT_W               float ×4（w/x/y/z）
 *   0x26       EULER_ROLL           float ×3（roll/pitch/yaw，rad）
 *   0x32       BARO_HEIGHT          float ×4（高度/温度/气压/气压差）
 *   0x61       ALGO_TYPE            uint8，读写（6=六轴，9=九轴）
 *   0x70       CALIB_IMU            uint8，写 1 开始，读非 0 表示完成
 *   0x71       CALIB_MAG            uint8，同上
 *
 * 单位换算（与厂家 Python 库一致）：
 *   加速度 g   = raw * 16 / 32767
 *   角速度 rad = raw * 2000 / 32767 * π / 180
 *   磁场 uT    = raw * 800 / 32767
 *
 * 数据更新方式：
 *   本驱动不使用中断，由 BSP_IMU_Task() 每 10 ms 主动读一组寄存器，
 *   6 组轮转（欧拉角→加速度→角速度→磁场→四元数→气压），
 *   即每组约 60 ms 刷新一次，不占用 I2C 中断/DMA。
 * ===================================================================== */

#define BSP_IMU_OK             0U
#define BSP_IMU_ERROR          1U

/* I2C 从机地址（7 位原值，驱动内部左移成 HAL 需要的 8 位形式） */
#define BSP_IMU_I2C_ADDR       0x23U

/* 寄存器地址（与厂家 I2C 协议表一致） */
#define BSP_IMU_REG_VERSION    0x01U
#define BSP_IMU_REG_ACCEL      0x04U
#define BSP_IMU_REG_GYRO       0x0AU
#define BSP_IMU_REG_MAG        0x10U
#define BSP_IMU_REG_QUAT       0x16U
#define BSP_IMU_REG_EULER      0x26U
#define BSP_IMU_REG_BARO       0x32U
#define BSP_IMU_REG_ALGO       0x61U
#define BSP_IMU_REG_CALIB_IMU  0x70U
#define BSP_IMU_REG_CALIB_MAG  0x71U

/* 传感器数据（单位已换算好，上层直接用） */
typedef struct {
    float   accel[3];        /* 加速度 g，索引 0/1/2 = x/y/z */
    float   gyro[3];         /* 角速度 rad/s */
    float   mag[3];          /* 磁场 uT */
    float   quat[4];         /* 四元数 w/x/y/z */
    float   euler[3];        /* 欧拉角 rad，索引 0/1/2 = roll/pitch/yaw */
    float   baro[4];         /* 高度 m / 温度 ℃ / 气压 / 气压差 */
    uint8_t version[3];      /* 固件版本 H/M/L */
    uint32_t frame_count;    /* 成功读取的组数，持续增长说明链路正常 */
    uint32_t err_count;      /* I2C 读失败次数（诊断用） */
    uint32_t last_tick;      /* 最近一次成功读取的时间戳 HAL_GetTick() */
} BSP_IMU_Data_t;

/* 作用：保存 I2C 句柄并读一次版本号做在线探测。
 * 参数 hi2c：传感器所在的 I2C 句柄（本板为 &hi2c1，PB7/PA15）。
 * 返回 BSP_IMU_OK 表示传感器有应答，BSP_IMU_ERROR 表示没读到 */
uint8_t BSP_IMU_Init(I2C_HandleTypeDef *hi2c);

/* 作用：周期任务，主循环里调用。内部按 10 ms 节流、6 组寄存器轮转读取，
 * 非阻塞时间很短（每次约 0.2~0.5 ms），读完更新 BSP_IMU_GetData 的内容 */
void BSP_IMU_Task(void);

/* 作用：取数据指针。返回 NULL 表示还没 Init。
 * 注意：返回的是驱动内部变量，上层只读，不要改 */
const BSP_IMU_Data_t *BSP_IMU_GetData(void);

/* 作用：判断传感器是否在线（最近 timeout_ms 内成功读到过数据）。
 * 参数 timeout_ms：超时时间，建议 200~500 ms */
uint8_t BSP_IMU_IsOnline(uint32_t timeout_ms);

/* 作用：设置融合算法类型（写 0x61）。
 * 参数 algo：6 = 六轴（不用磁力计），9 = 九轴（航向不漂），其它值按 9 处理。
 * 注意：模块会切换算法并重启输出，函数内等待 1 s 再返回 */
uint8_t BSP_IMU_SetAlgoType(uint8_t algo);

/* 作用：加速度/角速度零偏校准（写 0x70，然后轮询 0x70 直到非 0）。
 * 参数 timeout_ms：最长等待时间，建议 30000；校准期间模块必须静止。
 * 注意：阻塞等待，返回 BSP_IMU_OK 表示校准完成 */
uint8_t BSP_IMU_CalibrateIMU(uint32_t timeout_ms);

/* 作用：磁力计校准（写 0x71，然后轮询 0x71 直到非 0）。
 * 参数 timeout_ms：最长等待时间，建议 60000；校准期间要缓慢转动模块。
 * 注意：阻塞等待，返回 BSP_IMU_OK 表示校准完成 */
uint8_t BSP_IMU_CalibrateMag(uint32_t timeout_ms);

#endif
