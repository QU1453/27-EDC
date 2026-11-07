#include "lcd_port.h"

/* =====================================================================
 * lcd_port.c —— 屏幕端口层实现（Hardware 层，引脚 + SPI 的搬运工）
 *
 * 文件作用：
 *   把 HAL 的 SPI 发送和 GPIO 写引脚包成 LCD_xxx 函数，供 lcd.c 调用。
 *   换板子 / 换 SPI 时只改这里，绘图层一行都不用动。
 *
 * 命名对照：
 *   spi = Serial Peripheral Interface 串行外设接口
 *   tx = transmit 发送         buf/len = 缓冲/长度     n = 本次发送字节数
 *   hspi2 = SPI2 句柄（CubeMX 生成，只引用不重定义）
 *   Rst/Bl/Cs/Dc = 复位/背光/片选/数据命令选择
 *   WriteCmd = 写命令          WriteData = 写数据
 *   WriteDataHalfWord = 写 2 字节    WriteDataBulk = 批量写
 *   tmp = 临时变量（存字节交换结果）
 * ===================================================================== */

/* SPI2 由 CubeMX 生成（spi.c 里的 hspi2），这里只引用不重定义 */

/* 作用：把一段内存发到 SPI2。HAL 单次最多 0xFFFF 字节，超长自动分段。 */
static void lcd_spi_tx(const uint8_t *buf, uint32_t len)
{
    uint16_t n;

    /* HAL 单次最多 0xFFFF 字节，超长分段发送 */
    while (len > 0U) {
        n = (len > 0xFFFFU) ? 0xFFFFU : (uint16_t)len;
        (void)HAL_SPI_Transmit(&hspi2, (uint8_t *)buf, n, 1000U);
        buf += n;
        len -= n;
    }
}

/* 作用：端口初始化。片选拉常低、DC 置高（数据态），复位/背光交给 LCD_Init 的时序。 */
void LCD_PortInit(void)
{
    /* 片选常低（与例程一致），复位/背光电平由 LCD_Init 的时序控制 */
    LCD_CsPin(0);
    LCD_DcPin(1);
}

/* 作用：毫秒延时，直接转发 HAL_Delay。 */
void LCD_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

/* 作用：写复位脚电平（1 高 / 0 低）。 */
void LCD_RstPin(uint8_t level)
{
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 作用：写背光脚电平（1 点亮 / 0 熄灭）。 */
void LCD_BlPin(uint8_t level)
{
    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 作用：写片选脚电平。 */
void LCD_CsPin(uint8_t level)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 作用：写 DC 脚电平。0 = 后面是命令，1 = 后面是数据。 */
void LCD_DcPin(uint8_t level)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 作用：写命令：DC=0 + 发 1 字节。 */
void LCD_WriteCmd(uint8_t cmd)
{
    LCD_DcPin(0);
    lcd_spi_tx(&cmd, 1U);
}

/* 作用：写 8bit 数据：DC=1 + 发 1 字节。 */
void LCD_WriteData(uint8_t data)
{
    LCD_DcPin(1);
    lcd_spi_tx(&data, 1U);
}

/* 作用：写 16bit 数据（坐标/颜色）。屏幕要求高字节在前，而 HAL 按内存顺序发，
 *       故先交换字节；配合上层不再重复交换。 */
void LCD_WriteDataHalfWord(uint16_t data)
{
    uint16_t tmp = (uint16_t)((data << 8) | (data >> 8));

    LCD_DcPin(1);
    lcd_spi_tx((const uint8_t *)&tmp, 2U);
}

/* 作用：批量写数据。调用前 buf 里的字节顺序必须已经是「高字节在前」。 */
void LCD_WriteDataBulk(const uint8_t *buf, uint32_t len)
{
    LCD_DcPin(1);
    lcd_spi_tx(buf, len);
}
