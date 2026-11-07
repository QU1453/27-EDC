#ifndef __LCD_PORT_H
#define __LCD_PORT_H

#include <stdint.h>
#include "main.h"
#include "spi.h"

/* =====================================================================
 * lcd_port.h —— 屏幕端口层（Hardware 层，最贴近硬件的一层）
 *
 * 文件作用：
 *   把「拉某个引脚、往 SPI2 发几个字节」封装成 LCD_xxx 函数。
 *   上层（lcd.c）只调这些函数，完全不认识 GPIO / SPI 的名字，
 *   换板子时只改这一个文件和它的 .c，上层一行都不用动。
 *
 * 命名对照：
 *   Port = 端口（引脚 + 外设的搬运层）  Init = 初始化
 *   Delay = 毫秒延时         Rst = Reset 复位        Bl = BackLight 背光
 *   Cs = Chip Select 片选    Dc = Data/Command 数据/命令选择
 *   WriteCmd = 写命令        WriteData = 写 1 字节数据
 *   WriteDataHalfWord = 写 2 字节数据（半字）  WriteDataBulk = 批量写数据
 *   cmd = 命令字节           data = 数据字节         buf/len = 缓冲/长度
 *   level = 电平（1 高 / 0 低）   ms = 毫秒
 * ===================================================================== */

/* ============================ 屏幕接口 ============================
 *  屏上没有排针，只有一根 16pin / 0.5mm 间距的 FFC 软排线，
 *  插在板上的 FPC1 = FPC-05F-16PH20 座子上（17/18 脚为屏蔽/地，非信号）。
 *  屏侧为 22pin 的 FPC2 = AFC07-S22ECA-00，中间经转接板。
 *  排线内信号：SCK / SDA / DC / CS / RST / BLK / TE + VDD / GND / LEDK
 *  即 4 线单向 SPI（无 MISO），引脚 100% 由 PCB 走线决定，不可自选。
 * ======================== 引脚（原理图 / 厂家例程一致） ========================
 *   PA5  -> LCD_RST        复位（GPIO 推挽输出，低有效）
 *   PA6  -> LCD_BLK        背光（GPIO 推挽输出，高点亮）
 *   PA7  -> LCD_CS         片选（GPIO 推挽输出，常低）
 *   PB13 -> LCD_SCK        SPI2_SCK  (AF5)  板上网络 EXT_SPI2_SCK
 *   PB14 -> LCD_DC         数据/命令（GPIO 推挽输出，SPI2_MISO 复用）
 *   PB15 -> LCD_SDA        SPI2_MOSI (AF5)，单向 1LINE 模式
 *
 *  因为 PB14 被当 DC 用，SPI2 必须配成 Half-Duplex Master（1LINE），
 *  否则 CubeMX 会把 PB14 占成 SPI2_MISO。
 *  PB13 原来是 FDCAN2_TX，本工程只保留 FDCAN1（PA11/PA12），故已释放。
 *
 *  CubeMX 里给这 6 个脚打上 User Label（LCD_RST / LCD_BLK / LCD_CS /
 *  LCD_SCK / LCD_DC / LCD_SDA）后，main.h 会自动生成 LCD_xxx_Pin 宏；
 *  下面用 #ifndef 兜底，防止还没重新生成工程时编译不过。
 * ================================================================== */
#ifndef LCD_RST_Pin
#define LCD_RST_Pin         GPIO_PIN_5
#define LCD_RST_GPIO_Port   GPIOA
#endif
#ifndef LCD_BLK_Pin
#define LCD_BLK_Pin         GPIO_PIN_6
#define LCD_BLK_GPIO_Port   GPIOA
#endif
#ifndef LCD_CS_Pin
#define LCD_CS_Pin          GPIO_PIN_7
#define LCD_CS_GPIO_Port    GPIOA
#endif
#ifndef LCD_DC_Pin
#define LCD_DC_Pin          GPIO_PIN_14
#define LCD_DC_GPIO_Port    GPIOB
#endif

/* 作用：端口初始化。片选拉低、DC 置高，为后续发命令做准备。 */
void LCD_PortInit(void);

/* 作用：毫秒级延时。初始化时序要用，内部就是 HAL_Delay。 */
void LCD_Delay(uint32_t ms);

/* 作用：设置复位脚电平。 */
void LCD_RstPin(uint8_t level);

/* 作用：设置背光脚电平（1 = 点亮）。 */
void LCD_BlPin(uint8_t level);

/* 作用：设置片选脚电平（本工程常低）。 */
void LCD_CsPin(uint8_t level);

/* 作用：设置 DC 脚电平（0 = 接下来是命令，1 = 接下来是数据）。 */
void LCD_DcPin(uint8_t level);

/* 作用：写一个命令字节（DC=0 + SPI 发 1 字节）。 */
void LCD_WriteCmd(uint8_t cmd);

/* 作用：写一个数据字节（DC=1 + SPI 发 1 字节）。 */
void LCD_WriteData(uint8_t data);

/* 作用：写一个半字（坐标/颜色），内部先交换高低字节，屏幕要求高字节在前。 */
void LCD_WriteDataHalfWord(uint16_t data);

/* 作用：批量写数据。调用前 buf 里的字节顺序必须已经是「高字节在前」。 */
void LCD_WriteDataBulk(const uint8_t *buf, uint32_t len);

#endif
