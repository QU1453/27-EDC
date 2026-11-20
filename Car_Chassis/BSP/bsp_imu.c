/* =====================================================================
 * bsp_imu.c —— YbImu 姿态传感器驱动（BSP 层，I2C）
 *
 * 文件作用：
 *   I2C 主机主动轮询：主循环调用 BSP_IMU_Task，按 10 ms 节流依次读
 *   欧拉角 / 加速度 / 角速度 / 磁场 / 四元数 / 气压，换算成物理量后
 *   放进 BSP_IMU_Data_t，上层只管读。不改 CubeMX 生成的任何文件，
 *   只借用已初始化的 I2C1 句柄（PB7=SDA，PA15=SCL，从机地址 0x23）。
 *
 * 数据流：
 *   BSP_IMU_Task（主循环，10 ms 一次）
 *     → HAL_I2C_Mem_Read(0x23, reg, 1 字节寄存器地址, N 字节)
 *     → 小端解析：int16 / float32 → 物理量
 *     → s_data（时间戳、成功/失败计数）
 *
 * 为什么不用中断/DMA：
 *   每次只读 3~16 字节，阻塞时间不到 0.5 ms；轮询简单可靠，
 *   也避免和达妙串口中断、CAN 中断抢资源。
 * ===================================================================== */

#include "bsp_imu.h"

#include <string.h>

#define BSP_IMU_ADDR8          (BSP_IMU_I2C_ADDR << 1)   /* HAL 用 8 位地址 */
#define BSP_IMU_TIMEOUT_MS     5U                        /* 单次传输超时 */
#define BSP_IMU_POLL_MS        10U                       /* 轮询间隔 */
#define BSP_IMU_BUF_MAX        16U                       /* 最大一组 16 字节 */
#define BSP_IMU_CALIB_STEP_MS  100U                      /* 校准轮询间隔 */

/* 每次 Task 读哪一组，顺序轮转 */
enum {
    BSP_IMU_GRP_EULER = 0,
    BSP_IMU_GRP_ACCEL,
    BSP_IMU_GRP_GYRO,
    BSP_IMU_GRP_MAG,
    BSP_IMU_GRP_QUAT,
    BSP_IMU_GRP_BARO,
    BSP_IMU_GRP_NUM
};

/* 换算系数：厂家量程 16 g / 2000 dps / 800 uT，raw 为 int16 */
#define BSP_IMU_ACCEL_SCALE    (16.0f / 32767.0f)
#define BSP_IMU_GYRO_SCALE     (2000.0f / 32767.0f * 0.0174532925f)
#define BSP_IMU_MAG_SCALE      (800.0f / 32767.0f)

static I2C_HandleTypeDef *s_i2c;                 /* 传感器所在 I2C */
static BSP_IMU_Data_t     s_data;                /* 给上层的解析结果 */
static uint32_t           s_poll_tick;           /* 上次轮询时间 */
static uint8_t            s_group = BSP_IMU_GRP_EULER;  /* 下一组要读的 */

/* 作用：小端 16 位转有符号整数（原始量是 int16） */
static int16_t bsp_imu_le_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* 作用：小端 32 位转 float（姿态角/四元数是 float32）。
 * 用 memcpy 而不是指针强转，避开对齐和严格别名问题 */
static float bsp_imu_le_f32(const uint8_t *p)
{
    uint32_t raw = (uint32_t)p[0]
                 | ((uint32_t)p[1] << 8)
                 | ((uint32_t)p[2] << 16)
                 | ((uint32_t)p[3] << 24);
    float f;
    memcpy(&f, &raw, sizeof(f));
    return f;
}

/* 作用：从寄存器 reg 连读 len 字节，成功返回 BSP_IMU_OK */
static uint8_t bsp_imu_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (s_i2c == NULL) {
        return BSP_IMU_ERROR;
    }
    return (HAL_I2C_Mem_Read(s_i2c, BSP_IMU_ADDR8, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, len,
                             BSP_IMU_TIMEOUT_MS) == HAL_OK)
           ? BSP_IMU_OK : BSP_IMU_ERROR;
}

/* 作用：往寄存器 reg 写 1 字节，成功返回 BSP_IMU_OK */
static uint8_t bsp_imu_write(uint8_t reg, uint8_t value)
{
    if (s_i2c == NULL) {
        return BSP_IMU_ERROR;
    }
    return (HAL_I2C_Mem_Write(s_i2c, BSP_IMU_ADDR8, reg,
                              I2C_MEMADD_SIZE_8BIT, &value, 1U,
                              BSP_IMU_TIMEOUT_MS) == HAL_OK)
           ? BSP_IMU_OK : BSP_IMU_ERROR;
}

/* 作用：把三轴原始量（6 字节 int16）换算成物理量写进 out[3] */
static void bsp_imu_decode_vec3(const uint8_t *p, float *out, float scale)
{
    out[0] = (float)bsp_imu_le_i16(&p[0]) * scale;
    out[1] = (float)bsp_imu_le_i16(&p[2]) * scale;
    out[2] = (float)bsp_imu_le_i16(&p[4]) * scale;
}

/* 作用：读一组寄存器并解析。返回 BSP_IMU_OK 表示这次读成功 */
static uint8_t bsp_imu_read_group(uint8_t group)
{
    uint8_t buf[BSP_IMU_BUF_MAX];
    uint8_t ret = BSP_IMU_ERROR;

    switch (group) {
    case BSP_IMU_GRP_EULER:
        if (bsp_imu_read(BSP_IMU_REG_EULER, buf, 12U) == BSP_IMU_OK) {
            s_data.euler[0] = bsp_imu_le_f32(&buf[0]);
            s_data.euler[1] = bsp_imu_le_f32(&buf[4]);
            s_data.euler[2] = bsp_imu_le_f32(&buf[8]);
            ret = BSP_IMU_OK;
        }
        break;

    case BSP_IMU_GRP_ACCEL:
        if (bsp_imu_read(BSP_IMU_REG_ACCEL, buf, 6U) == BSP_IMU_OK) {
            bsp_imu_decode_vec3(buf, s_data.accel, BSP_IMU_ACCEL_SCALE);
            ret = BSP_IMU_OK;
        }
        break;

    case BSP_IMU_GRP_GYRO:
        if (bsp_imu_read(BSP_IMU_REG_GYRO, buf, 6U) == BSP_IMU_OK) {
            bsp_imu_decode_vec3(buf, s_data.gyro, BSP_IMU_GYRO_SCALE);
            ret = BSP_IMU_OK;
        }
        break;

    case BSP_IMU_GRP_MAG:
        if (bsp_imu_read(BSP_IMU_REG_MAG, buf, 6U) == BSP_IMU_OK) {
            bsp_imu_decode_vec3(buf, s_data.mag, BSP_IMU_MAG_SCALE);
            ret = BSP_IMU_OK;
        }
        break;

    case BSP_IMU_GRP_QUAT:
        if (bsp_imu_read(BSP_IMU_REG_QUAT, buf, 16U) == BSP_IMU_OK) {
            s_data.quat[0] = bsp_imu_le_f32(&buf[0]);
            s_data.quat[1] = bsp_imu_le_f32(&buf[4]);
            s_data.quat[2] = bsp_imu_le_f32(&buf[8]);
            s_data.quat[3] = bsp_imu_le_f32(&buf[12]);
            ret = BSP_IMU_OK;
        }
        break;

    case BSP_IMU_GRP_BARO:
        if (bsp_imu_read(BSP_IMU_REG_BARO, buf, 16U) == BSP_IMU_OK) {
            s_data.baro[0] = bsp_imu_le_f32(&buf[0]);
            s_data.baro[1] = bsp_imu_le_f32(&buf[4]);
            s_data.baro[2] = bsp_imu_le_f32(&buf[8]);
            s_data.baro[3] = bsp_imu_le_f32(&buf[12]);
            ret = BSP_IMU_OK;
        }
        break;

    default:
        break;
    }

    return ret;
}

/* 作用：保存 I2C 句柄，读一次版本号确认传感器有应答 */
uint8_t BSP_IMU_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t ret;

    if (hi2c == NULL) {
        return BSP_IMU_ERROR;
    }

    s_i2c       = hi2c;
    s_group     = BSP_IMU_GRP_EULER;
    s_poll_tick = HAL_GetTick() - BSP_IMU_POLL_MS;
    memset(&s_data, 0, sizeof(s_data));

    ret = bsp_imu_read(BSP_IMU_REG_VERSION, s_data.version, 3U);
    if (ret == BSP_IMU_OK) {
        s_data.frame_count = 1U;
        s_data.last_tick   = HAL_GetTick();
    }

    return ret;
}

/* 作用：10 ms 节流读一组寄存器，6 组轮转（主循环调用，非阻塞轮询） */
void BSP_IMU_Task(void)
{
    uint32_t now;

    if (s_i2c == NULL) {
        return;
    }

    now = HAL_GetTick();
    if ((now - s_poll_tick) < BSP_IMU_POLL_MS) {
        return;
    }
    s_poll_tick = now;

    if (bsp_imu_read_group(s_group) == BSP_IMU_OK) {
        s_data.frame_count++;
        s_data.last_tick = now;
    } else {
        s_data.err_count++;
    }

    s_group = (uint8_t)((s_group + 1U) % BSP_IMU_GRP_NUM);
}

/* 作用：给上层取解析结果 */
const BSP_IMU_Data_t *BSP_IMU_GetData(void)
{
    return (s_i2c != NULL) ? &s_data : NULL;
}

/* 作用：判断最近 timeout_ms 内有没有成功读到数据 */
uint8_t BSP_IMU_IsOnline(uint32_t timeout_ms)
{
    if ((s_i2c == NULL) || (s_data.frame_count == 0U)) {
        return 0U;
    }
    return ((HAL_GetTick() - s_data.last_tick) < timeout_ms) ? 1U : 0U;
}

/* 作用：设置融合算法类型（写 0x61），模块切换算法约需 1 s */
uint8_t BSP_IMU_SetAlgoType(uint8_t algo)
{
    uint8_t ret;

    if (algo != 6U) {
        algo = 9U;
    }

    ret = bsp_imu_write(BSP_IMU_REG_ALGO, algo);
    if (ret == BSP_IMU_OK) {
        HAL_Delay(1000U);      /* 等模块按新算法重新起算 */
    }
    return ret;
}

/* 作用：轮询校准寄存器直到完成。写 1 后模块把该寄存器清 0，完成后写非 0 */
static uint8_t bsp_imu_wait_calib(uint8_t reg, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t  value = 0U;

    if (bsp_imu_write(reg, 0x01U) != BSP_IMU_OK) {
        return BSP_IMU_ERROR;
    }

    while ((HAL_GetTick() - start) < timeout_ms) {
        HAL_Delay(BSP_IMU_CALIB_STEP_MS);
        if ((bsp_imu_read(reg, &value, 1U) == BSP_IMU_OK) && (value != 0U)) {
            return BSP_IMU_OK;
        }
    }
    return BSP_IMU_ERROR;
}

/* 作用：加速度/角速度零偏校准，模块保持静止 */
uint8_t BSP_IMU_CalibrateIMU(uint32_t timeout_ms)
{
    return bsp_imu_wait_calib(BSP_IMU_REG_CALIB_IMU, timeout_ms);
}

/* 作用：磁力计校准，期间缓慢转动模块 */
uint8_t BSP_IMU_CalibrateMag(uint32_t timeout_ms)
{
    return bsp_imu_wait_calib(BSP_IMU_REG_CALIB_MAG, timeout_ms);
}
