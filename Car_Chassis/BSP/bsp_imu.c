/* =====================================================================
 * bsp_imu.c —— YbImu 姿态传感器驱动（BSP 层，UART）
 *
 * 文件作用：
 *   串口中断逐字节收数 → 环形缓冲区 → BSP_IMU_Task 里按协议拆帧
 *   → 解析成 BSP_IMU_Data_t，上层只管读。不改 CubeMX 生成的任何文件，
 *   只借用已初始化的串口句柄和它的接收中断。
 *
 * 数据流：
 *   USARTx_IRQHandler（CubeMX 生成）
 *     → HAL_UART_IRQHandler
 *       → HAL_UART_RxCpltCallback（本文件）把 1 字节压入环形缓冲区并重新挂中断
 *   BSP_IMU_Task（主循环）
 *     → 状态机 0x7E → 0x23 → len → func → data... → sum
 *     → 按 func 解出原始量/四元数/欧拉角/气压
 *
 * 帧示例（欧拉角，上报 yaw=0 时）：
 *   7E 23 11 26 | 00 00 00 00 | 00 00 00 00 | 00 00 00 00 | sum
 *   └head──┘ len func └──roll────┘└──pitch───┘└───yaw────┘  └校验
 * ===================================================================== */

#include "bsp_imu.h"
#include "bsp_dm_uart.h"

#include <string.h>

#define BSP_IMU_RING_SIZE      512U    /* 必须是 2 的幂，接收环形缓冲区 */
#define BSP_IMU_RING_MASK      (BSP_IMU_RING_SIZE - 1U)
#define BSP_IMU_PAYLOAD_MAX    32U     /* 最大载荷 18 字节，留余量 */
#define BSP_IMU_FRAME_MIN      5U      /* 最短帧：头 2 + len + func + 校验 */
#define BSP_IMU_FRAME_MAX      40U     /* 厂家协议上限 */
#define BSP_IMU_CMD_TIMEOUT_MS 20U     /* 发命令阻塞超时，7 字节约 0.6 ms */

/* 解析状态机 */
enum {
    BSP_IMU_ST_HEAD1 = 0,   /* 等 0x7E */
    BSP_IMU_ST_HEAD2,       /* 等 0x23 */
    BSP_IMU_ST_LEN,         /* 取整帧长度 */
    BSP_IMU_ST_FUNC,        /* 取功能字 */
    BSP_IMU_ST_DATA,        /* 收载荷 */
    BSP_IMU_ST_SUM          /* 收校验 */
};

static UART_HandleTypeDef *s_uart;              /* 传感器所在的串口 */
static uint8_t  s_rx_byte;                      /* 中断单字节暂存 */
static uint8_t  s_ring[BSP_IMU_RING_SIZE];      /* 接收环形缓冲区 */
static volatile uint16_t s_ring_head;           /* 中断写 */
static volatile uint16_t s_ring_tail;           /* 主循环读 */

static uint8_t  s_state = BSP_IMU_ST_HEAD1;     /* 状态机当前状态 */
static uint8_t  s_len;                          /* 本次帧的整帧长度 */
static uint8_t  s_func;                         /* 本次帧的功能字 */
static uint8_t  s_payload[BSP_IMU_PAYLOAD_MAX]; /* 载荷暂存 */
static uint8_t  s_index;                        /* 载荷已收字节数 */
static uint8_t  s_sum;                          /* 累加和 */

static BSP_IMU_Data_t s_data;                   /* 给上层的解析结果 */

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

/* 作用：按功能字填 s_data。只有校验正确的整帧才会走到这里 */
static void bsp_imu_decode(uint8_t func, const uint8_t *p, uint8_t len)
{
    switch (func) {
    case BSP_IMU_FUNC_RAW:      /* 9 个 int16：加速度 g、角速度 rad/s、磁场 uT */
        if (len >= 18U) {
            s_data.accel[0] = (float)bsp_imu_le_i16(&p[0])  * (16.0f / 32767.0f);
            s_data.accel[1] = (float)bsp_imu_le_i16(&p[2])  * (16.0f / 32767.0f);
            s_data.accel[2] = (float)bsp_imu_le_i16(&p[4])  * (16.0f / 32767.0f);
            s_data.gyro[0]  = (float)bsp_imu_le_i16(&p[6])  * (2000.0f / 32767.0f * 0.0174532925f);
            s_data.gyro[1]  = (float)bsp_imu_le_i16(&p[8])  * (2000.0f / 32767.0f * 0.0174532925f);
            s_data.gyro[2]  = (float)bsp_imu_le_i16(&p[10]) * (2000.0f / 32767.0f * 0.0174532925f);
            s_data.mag[0]   = (float)bsp_imu_le_i16(&p[12]) * (800.0f / 32767.0f);
            s_data.mag[1]   = (float)bsp_imu_le_i16(&p[14]) * (800.0f / 32767.0f);
            s_data.mag[2]   = (float)bsp_imu_le_i16(&p[16]) * (800.0f / 32767.0f);
        }
        break;

    case BSP_IMU_FUNC_QUAT:     /* 4 个 float：w x y z */
        if (len >= 16U) {
            s_data.quat[0] = bsp_imu_le_f32(&p[0]);
            s_data.quat[1] = bsp_imu_le_f32(&p[4]);
            s_data.quat[2] = bsp_imu_le_f32(&p[8]);
            s_data.quat[3] = bsp_imu_le_f32(&p[12]);
        }
        break;

    case BSP_IMU_FUNC_EULER:    /* 3 个 float：roll pitch yaw，rad */
        if (len >= 12U) {
            s_data.euler[0] = bsp_imu_le_f32(&p[0]);
            s_data.euler[1] = bsp_imu_le_f32(&p[4]);
            s_data.euler[2] = bsp_imu_le_f32(&p[8]);
        }
        break;

    case BSP_IMU_FUNC_BARO:     /* 4 个 float：高度 m、温度 ℃、气压、气压差 */
        if (len >= 16U) {
            s_data.baro[0] = bsp_imu_le_f32(&p[0]);
            s_data.baro[1] = bsp_imu_le_f32(&p[4]);
            s_data.baro[2] = bsp_imu_le_f32(&p[8]);
            s_data.baro[3] = bsp_imu_le_f32(&p[12]);
        }
        break;

    case BSP_IMU_FUNC_VERSION:  /* 3 字节：主/次/修订 */
        if (len >= 3U) {
            s_data.version[0] = p[0];
            s_data.version[1] = p[1];
            s_data.version[2] = p[2];
        }
        break;

    default:
        break;
    }
}

/* 作用：喂 1 字节给状态机，凑齐一帧且校验通过就解析 */
static void bsp_imu_parse_byte(uint8_t b)
{
    switch (s_state) {
    case BSP_IMU_ST_HEAD1:
        if (b == 0x7EU) {
            s_state = BSP_IMU_ST_HEAD2;
        }
        break;

    case BSP_IMU_ST_HEAD2:
        s_state = (b == 0x23U) ? BSP_IMU_ST_LEN : BSP_IMU_ST_HEAD1;
        break;

    case BSP_IMU_ST_LEN:
        if ((b >= BSP_IMU_FRAME_MIN) && (b <= BSP_IMU_FRAME_MAX)) {
            s_len    = b;
            s_state  = BSP_IMU_ST_FUNC;
        } else {
            s_state = BSP_IMU_ST_HEAD1;
        }
        break;

    case BSP_IMU_ST_FUNC:
        s_func  = b;
        s_index = 0U;
        s_sum   = (uint8_t)(0x7EU + 0x23U + s_len + s_func);
        s_state = (s_len > 5U) ? BSP_IMU_ST_DATA : BSP_IMU_ST_SUM;
        break;

    case BSP_IMU_ST_DATA:       /* 载荷长度 = 整帧长度 - 头2 - len - func - 校验 */
        s_payload[s_index++] = b;
        s_sum = (uint8_t)(s_sum + b);
        if (s_index >= (uint8_t)(s_len - 5U)) {
            s_state = BSP_IMU_ST_SUM;
        }
        break;

    case BSP_IMU_ST_SUM:
        if (b == s_sum) {
            bsp_imu_decode(s_func, s_payload, s_index);
            s_data.last_tick = HAL_GetTick();
            s_data.frame_count++;
        }
        s_state = BSP_IMU_ST_HEAD1;
        break;

    default:
        s_state = BSP_IMU_ST_HEAD1;
        break;
    }
}

/* 作用：组一帧 7 字节命令发出去（7E 23 07 func p0 p1 sum）。
 * 参数 func：功能字；p0/p1：两个参数字节。返回 BSP_IMU_OK 表示发成功 */
static uint8_t bsp_imu_send_cmd(uint8_t func, uint8_t p0, uint8_t p1)
{
    uint8_t frame[8];
    uint8_t sum = 0U;
    uint8_t i;

    if (s_uart == NULL) {
        return BSP_IMU_ERROR;
    }

    frame[0] = 0x7EU;
    frame[1] = 0x23U;
    frame[2] = 0x07U;
    frame[3] = func;
    frame[4] = p0;
    frame[5] = p1;

    for (i = 0U; i < 6U; i++) {
        sum = (uint8_t)(sum + frame[i]);
    }
    frame[6] = sum;

    return (HAL_UART_Transmit(s_uart, frame, 7U, BSP_IMU_CMD_TIMEOUT_MS) == HAL_OK)
           ? BSP_IMU_OK : BSP_IMU_ERROR;
}

/* 作用：挂上单字节接收中断。若 HAL 内部状态被上一次错误卡住（返回 HAL_BUSY），
 * 先强制中止接收再挂一次，否则中断只进一次就再也不来了（表现为 RX 停在 1） */
static void bsp_imu_arm_rx(void)
{
    if (s_uart == NULL) {
        return;
    }
    if (HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U) != HAL_OK) {
        (void)HAL_UART_AbortReceive(s_uart);
        (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1U);
    }
}

/* 作用：挂上串口接收中断并清空驱动状态 */
uint8_t BSP_IMU_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return BSP_IMU_ERROR;
    }

    s_uart       = huart;
    s_ring_head  = 0U;
    s_ring_tail  = 0U;
    s_state      = BSP_IMU_ST_HEAD1;
    s_index      = 0U;
    s_len        = 0U;
    s_func       = 0U;
    s_sum        = 0U;
    memset(&s_data, 0, sizeof(s_data));

    bsp_imu_arm_rx();

    return (s_uart->RxState != HAL_UART_STATE_RESET) ? BSP_IMU_OK : BSP_IMU_ERROR;
}

/* 作用：把环形缓冲区里的字节全部喂给解析器（主循环调用，非阻塞） */
void BSP_IMU_Task(void)
{
    while (s_ring_tail != s_ring_head) {
        uint8_t b = s_ring[s_ring_tail];
        s_ring_tail = (uint16_t)((s_ring_tail + 1U) & BSP_IMU_RING_MASK);
        bsp_imu_parse_byte(b);
    }
}

/* 作用：给上层取解析结果 */
const BSP_IMU_Data_t *BSP_IMU_GetData(void)
{
    return (s_uart != NULL) ? &s_data : NULL;
}

/* 作用：判断最近 timeout_ms 内有没有收到有效帧 */
uint8_t BSP_IMU_IsOnline(uint32_t timeout_ms)
{
    if ((s_uart == NULL) || (s_data.frame_count == 0U)) {
        return 0U;
    }
    return ((HAL_GetTick() - s_data.last_tick) < timeout_ms) ? 1U : 0U;
}

/* 作用：设置上报频率（10~100 Hz，超范围夹紧） */
uint8_t BSP_IMU_SetReportRate(uint8_t hz)
{
    if (hz < 10U)  { hz = 10U;  }
    if (hz > 100U) { hz = 100U; }
    return bsp_imu_send_cmd(BSP_IMU_FUNC_RATE, hz, 0x5FU);
}

/* 作用：设置融合算法类型（6 轴 / 9 轴，非法值按 9 轴处理） */
uint8_t BSP_IMU_SetAlgoType(uint8_t algo)
{
    if (algo != 6U) { algo = 9U; }
    return bsp_imu_send_cmd(BSP_IMU_FUNC_ALGO, algo, 0x5FU);
}

/* 作用：主动请求一帧数据（7E 23 07 80 func 00 sum）。
 * 模块默认静默时，用它触发一次上报；返回 BSP_IMU_OK 表示发成功 */
uint8_t BSP_IMU_RequestData(uint8_t func)
{
    return bsp_imu_send_cmd(BSP_IMU_FUNC_REQUEST, func, 0x00U);
}

/* 作用：串口收满 1 字节中断回调。压入环形缓冲区后立刻重新挂中断，
 * 保证 100 Hz 上报不丢帧。缓冲区满就丢弃新字节（等下一次上报） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 达妙调试串口（USART3/UART4）先认领，不是它的才进 IMU 分支 */
    if (BSP_DM_UART_OnRx(huart) != 0U) {
        return;
    }

    if (huart == s_uart) {
        uint16_t next = (uint16_t)((s_ring_head + 1U) & BSP_IMU_RING_MASK);
        if (next != s_ring_tail) {
            s_ring[s_ring_head] = s_rx_byte;
            s_ring_head = next;
        }
        s_data.byte_count++;
        bsp_imu_arm_rx();
    }
}

/* 作用：串口出错（溢出/噪声/帧错）后重新挂接收中断，否则会一直收不到数据 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (BSP_DM_UART_OnError(huart) != 0U) {
        return;
    }
    if (huart == s_uart) {
        bsp_imu_arm_rx();
    }
}
