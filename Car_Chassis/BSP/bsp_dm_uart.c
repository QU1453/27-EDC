/* =====================================================================
 * bsp_dm_uart.c —— 达妙 DM3520-1EC 驱动器调试串口（USART3 / UART4）
 *
 * 为什么要在 BSP 里重初始化：
 *   CubeMX 生成的 usart.c 里两个口都是 115200，而达妙配置口固定 921600，
 *   生成文件不允许手改，所以在 Init 里把 Init 结构体改好再调一次 HAL_UART_Init。
 *
 * 收：HAL_UART_Receive_IT 单字节 → 256 B 环形缓冲 + 一行 ASCII 文本缓存
 * 发：阻塞发送（配置口只发几十字节的参数帧，不需要 DMA）
 * ===================================================================== */
#include "bsp_dm_uart.h"
#include "usart.h"

#include <string.h>

#define DM_RING_SIZE   256U
#define DM_RING_MASK   (DM_RING_SIZE - 1U)
#define DM_LINE_MAX    80U

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t   ring[DM_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t   rx_byte;
    volatile uint32_t rx_count;
    volatile uint8_t  inited;
    char      line[DM_LINE_MAX];
    uint8_t   line_len;
    char      last_line[DM_LINE_MAX];
    volatile uint32_t line_tick;
} bsp_dm_port_t;

static bsp_dm_port_t s_port[BSP_DM_UART_NUM] = {
    { .huart = &huart3 },   /* PB10 TX / PB11 RX */
    { .huart = &huart4 },   /* PC10 TX / PC11 RX */
};

static bsp_dm_port_t *dm_find(UART_HandleTypeDef *huart)
{
    uint8_t i;

    for (i = 0U; i < BSP_DM_UART_NUM; i++) {
        if (s_port[i].huart == huart) {
            return &s_port[i];
        }
    }
    return 0;
}

/* 作用：挂 1 字节中断接收。HAL_BUSY 说明上一次接收还挂着，先 Abort 再重挂 */
static void dm_arm(bsp_dm_port_t *p)
{
    if (HAL_UART_Receive_IT(p->huart, &p->rx_byte, 1U) != HAL_OK) {
        (void)HAL_UART_AbortReceive(p->huart);
        (void)HAL_UART_Receive_IT(p->huart, &p->rx_byte, 1U);
    }
}

void BSP_DM_UART_Init(void)
{
    uint8_t i;

    for (i = 0U; i < BSP_DM_UART_NUM; i++) {
        bsp_dm_port_t *p = &s_port[i];

        /* 只改波特率，其余 8N1 沿用 CubeMX 的配置 */
        p->huart->Init.BaudRate   = BSP_DM_UART_BAUD;
        p->huart->Init.WordLength = UART_WORDLENGTH_8B;
        p->huart->Init.StopBits   = UART_STOPBITS_1;
        p->huart->Init.Parity     = UART_PARITY_NONE;
        p->huart->Init.Mode       = UART_MODE_TX_RX;
        (void)HAL_UART_Init(p->huart);

        p->head     = 0U;
        p->tail     = 0U;
        p->rx_count = 0U;
        p->line_len = 0U;
        p->line[0]  = '\0';
        p->last_line[0] = '\0';
        p->inited   = 1U;
        dm_arm(p);
    }
}

uint16_t BSP_DM_UART_Available(uint8_t port)
{
    if (port >= BSP_DM_UART_NUM) {
        return 0U;
    }
    return (uint16_t)((s_port[port].head - s_port[port].tail) & DM_RING_MASK);
}

uint16_t BSP_DM_UART_Read(uint8_t port, uint8_t *buf, uint16_t max)
{
    uint16_t n = 0U;
    bsp_dm_port_t *p;

    if ((port >= BSP_DM_UART_NUM) || (buf == 0)) {
        return 0U;
    }
    p = &s_port[port];

    while ((n < max) && (p->tail != p->head)) {
        buf[n++] = p->ring[p->tail];
        p->tail  = (uint16_t)((p->tail + 1U) & DM_RING_MASK);
    }
    return n;
}

uint32_t BSP_DM_UART_RxCount(uint8_t port)
{
    if (port >= BSP_DM_UART_NUM) {
        return 0U;
    }
    return s_port[port].rx_count;
}

uint16_t BSP_DM_UART_GetLine(uint8_t port, char *out, uint16_t max)
{
    uint16_t n = 0U;

    if ((port >= BSP_DM_UART_NUM) || (out == 0) || (max == 0U)) {
        return 0U;
    }
    while ((s_port[port].last_line[n] != '\0') && ((uint16_t)(n + 1U) < max)) {
        out[n] = s_port[port].last_line[n];
        n++;
    }
    out[n] = '\0';
    return n;
}

uint32_t BSP_DM_UART_LineTick(uint8_t port)
{
    if (port >= BSP_DM_UART_NUM) {
        return 0U;
    }
    return s_port[port].line_tick;
}

uint8_t BSP_DM_UART_Send(uint8_t port, const uint8_t *buf, uint16_t len)
{
    if ((port >= BSP_DM_UART_NUM) || (buf == 0) || (len == 0U)) {
        return 1U;
    }
    return (HAL_UART_Transmit(s_port[port].huart, (uint8_t *)buf, len, 50U) == HAL_OK) ? 0U : 1U;
}

uint8_t BSP_DM_UART_OnRx(UART_HandleTypeDef *huart)
{
    bsp_dm_port_t *p = dm_find(huart);
    uint8_t  b;
    uint16_t next;

    if ((p == 0) || (p->inited == 0U)) {
        return 0U;
    }

    b    = p->rx_byte;
    next = (uint16_t)((p->head + 1U) & DM_RING_MASK);
    if (next != p->tail) {
        p->ring[p->head] = b;
        p->head = next;
    }
    p->rx_count++;

    /* 顺带攒一行可打印文本：驱动器上电会打印固件版本 / CAN ID / 波特率 / 电压 */
    if (b == (uint8_t)'\n') {
        p->line[p->line_len] = '\0';
        if (p->line_len > 0U) {
            (void)memcpy(p->last_line, p->line, (size_t)p->line_len + 1U);
            p->line_tick = HAL_GetTick();
        }
        p->line_len = 0U;
    } else if (b != (uint8_t)'\r') {
        if (p->line_len < (DM_LINE_MAX - 1U)) {
            p->line[p->line_len] = (char)b;
            p->line_len++;
        } else {
            p->line_len = 0U;   /* 超长行直接丢弃，避免半行误导 */
        }
    }

    dm_arm(p);
    return 1U;
}

uint8_t BSP_DM_UART_OnError(UART_HandleTypeDef *huart)
{
    bsp_dm_port_t *p = dm_find(huart);

    if ((p == 0) || (p->inited == 0U)) {
        return 0U;
    }
    dm_arm(p);
    return 1U;
}
