#include "bsp_can.h"
#include "fdcan.h"

/* =====================================================================
 * bsp_can.c —— CAN 收发的实现（BSP 层）
 *
 * 文件作用：
 *   实现 bsp_can.h 的三个函数，内部只调用 HAL_FDCAN_xxx。
 *
 * 命名对照：
 *   hfdcan1    = FDCAN1 的 HAL 句柄（CubeMX 在 fdcan.c 里定义，这里 extern 引用）
 *   filter     = 过滤器配置结构体
 *   tx / rx    = 发送 / 接收帧头结构体
 *   FilterID1  = 过滤基准 ID       FilterID2 = 过滤掩码
 *   FIFO0      = 接收队列 0
 * ===================================================================== */

/* 作用：配全局过滤 + （必要时）配过滤表 + 启动 FDCAN1 */
uint8_t BSP_CAN_Init(void)
{
  FDCAN_FilterTypeDef filter = {0};

  /* 不匹配的帧全部进 FIFO0；远程帧一律拒收（DM 和 C610 都不用远程帧） */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    return BSP_CAN_ERROR;
  }

  /* CubeMX 里 StdFiltersNbr=0 时过滤表是空的，全靠全局过滤全收；
   * 只有配了 StdFiltersNbr >= 1 才需要填过滤表，否则 HAL 会返回错误 */
  if (hfdcan1.Init.StdFiltersNbr > 0U)
  {
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0U;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000U;   /* 基准 ID */
    filter.FilterID2    = 0x000U;   /* 掩码全 0 = 全收 */
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
      return BSP_CAN_ERROR;
    }
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    return BSP_CAN_ERROR;
  }
  return BSP_CAN_OK;
}

/* 作用：把 8 字节按标准帧塞进发送 FIFO。
 * 所有格式字段都写死成"经典 CAN + 数据帧 + 8 字节"，
 * 因为 BRS / FDFormat 一旦误开，电机那头会直接收不到 */
uint8_t BSP_CAN_Send(uint32_t std_id, const uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef tx = {0};

  tx.Identifier         = std_id;
  tx.IdType             = FDCAN_STANDARD_ID;
  tx.TxFrameType        = FDCAN_DATA_FRAME;
  tx.DataLength         = FDCAN_DLC_BYTES_8;
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch      = FDCAN_BRS_OFF;
  tx.FDFormat           = FDCAN_CLASSIC_CAN;
  tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker      = 0U;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data) != HAL_OK)
  {
    return BSP_CAN_ERROR;
  }
  return BSP_CAN_OK;
}

/* 作用：从 FIFO0 取一帧。先查填充数量，空了就直接返回，避免白调 HAL */
uint8_t BSP_CAN_Recv(uint32_t *std_id, uint8_t data[8])
{
  FDCAN_RxHeaderTypeDef rx;

  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U)
  {
    return BSP_CAN_ERROR;
  }
  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, data) != HAL_OK)
  {
    return BSP_CAN_ERROR;
  }
  if (std_id != 0)
  {
    *std_id = rx.Identifier;
  }
  return BSP_CAN_OK;
}
