/* =====================================================================
 * app_imu.c —— 姿态传感器任务（App 层，板载 LCD 显示）
 *
 * 接线（115200 8N1，必须交叉）：
 *   传感器 TX  → PE1（USART1_RX）
 *   传感器 RX  → PE0（USART1_TX）
 *   传感器 GND → 板 GND
 *   传感器 5V  → 5V（板上无 5V 轨，由外部供给）
 *
 * 显示（320x240，左上角）：roll / pitch / yaw（度）、z 轴角速度、在线状态、
 * 帧计数。刷新用 LCD_ShowStringFast 整行批量发送，约 5 ms / 次。
 * ===================================================================== */

#include "app_imu.h"
#include "bsp_imu.h"
#include "usart.h"
#include "lcd.h"

#include <stdio.h>

/* ---------------- 可配置项 ---------------- */
#define APP_IMU_UART            (&huart1)   /* PE0=TX / PE1=RX，CubeMX 已初始化为 115200 */
#define APP_IMU_PERIOD_MS       250U        /* 屏幕刷新周期，4 Hz */
#define APP_IMU_ONLINE_MS       500U        /* 超过 500 ms 没帧算掉线 */
#define APP_IMU_REQ_PERIOD_MS   300U        /* 掉线时主动请求数据的间隔 */
#define APP_IMU_RAD2DEG         57.29578f

/* 屏幕布局（像素） */
#define APP_IMU_COL_X           32U         /* 数值列 */
#define APP_IMU_ROW_R           32U
#define APP_IMU_ROW_P           60U
#define APP_IMU_ROW_Y           88U
#define APP_IMU_ROW_G           116U
#define APP_IMU_ROW_STATE       160U
#define APP_IMU_ROW_CNT         184U
/* ------------------------------------------ */

static uint32_t s_last_tick;            /* 上次刷屏时间 */
static uint32_t s_req_tick;             /* 上次主动请求时间 */
static uint8_t  s_req_index;            /* 请求功能字轮转序号 */
static uint8_t  s_online;               /* 上次画的状态，0xFF = 还没画过 */

static void app_imu_draw_static(void)
{
    LCD_ShowString(0U, 0U, "IMU YbImu  USART1 115200", CYAN, BLACK);
    LCD_ShowString(0U, APP_IMU_ROW_R, "R:", GREEN, BLACK);
    LCD_ShowString(0U, APP_IMU_ROW_P, "P:", GREEN, BLACK);
    LCD_ShowString(0U, APP_IMU_ROW_Y, "Y:", GREEN, BLACK);
    LCD_ShowString(0U, APP_IMU_ROW_G, "GZ:", GREEN, BLACK);
}

/* 定宽输出，刷新不留残影 */
static void app_imu_draw_value(uint16_t y, float value)
{
    char text[16];

    (void)snprintf(text, sizeof(text), "%+7.2f", value);
    LCD_ShowStringFast(APP_IMU_COL_X, y, text, YELLOW, BLACK);
}

static void app_imu_refresh(void)
{
    const BSP_IMU_Data_t *d = BSP_IMU_GetData();
    uint8_t online;
    char text[40];

    if (d == NULL) {
        return;
    }

    online = BSP_IMU_IsOnline(APP_IMU_ONLINE_MS);

    if (online != 0U) {
        app_imu_draw_value(APP_IMU_ROW_R, d->euler[0] * APP_IMU_RAD2DEG);
        app_imu_draw_value(APP_IMU_ROW_P, d->euler[1] * APP_IMU_RAD2DEG);
        app_imu_draw_value(APP_IMU_ROW_Y, d->euler[2] * APP_IMU_RAD2DEG);

        (void)snprintf(text, sizeof(text), "%+7.3f", d->gyro[2]);
        LCD_ShowStringFast(APP_IMU_COL_X, APP_IMU_ROW_G, text, CYAN, BLACK);
    }

    /* 帧计数：F 持续增长说明链路正常 */
    (void)snprintf(text, sizeof(text), "F:%-6u RX:%-6u",
                   (unsigned)d->frame_count, (unsigned)d->byte_count);
    LCD_ShowStringFast(0U, APP_IMU_ROW_CNT, text, GRAY, BLACK);

    /* 状态行只在变化时写，省 SPI 带宽；两条字符串等长，不留残影 */
    if (online != s_online) {
        s_online = online;
        if (online != 0U) {
            LCD_ShowStringFast(0U, APP_IMU_ROW_STATE, "IMU OK      ", GREEN, BLACK);
        } else {
            LCD_ShowStringFast(0U, APP_IMU_ROW_STATE, "IMU NO DATA ", RED, BLACK);
        }
    }
}

/* 没数据时主动请一帧，触发模块上报；有帧后自动停发 */
static void app_imu_poll(void)
{
    static const uint8_t req_func[2] = { BSP_IMU_FUNC_EULER, BSP_IMU_FUNC_RAW };
    uint32_t now = HAL_GetTick();

    if ((now - s_req_tick) < APP_IMU_REQ_PERIOD_MS) {
        return;
    }
    s_req_tick = now;

    (void)BSP_IMU_RequestData(req_func[s_req_index]);
    s_req_index = (uint8_t)((s_req_index + 1U) & 0x01U);
}

void App_IMU_Init(void)
{
    s_online    = 0xFFU;
    s_last_tick = HAL_GetTick();
    s_req_tick  = HAL_GetTick();
    s_req_index = 0U;

    app_imu_draw_static();
    (void)BSP_IMU_Init(APP_IMU_UART);
    app_imu_refresh();
}

void App_IMU_Task(void)
{
    uint32_t now;

    BSP_IMU_Task();

    now = HAL_GetTick();

    if (BSP_IMU_IsOnline(APP_IMU_ONLINE_MS) == 0U) {
        app_imu_poll();
    }

    if ((now - s_last_tick) >= APP_IMU_PERIOD_MS) {
        s_last_tick = now;
        app_imu_refresh();
    }
}
