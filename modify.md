# car_dipan 修改记录（电赛小车底盘 / STM32G473VET6）

## 2026-10-18　屏幕驱动（Hardware 层）+ 显示“我是G473”

**目的**：加入 ST7789 2.00 寸（320×240）屏驱动，上电显示一行字。驱动独立放 Hardware 层，不改 CubeMX 生成逻辑。

**新增文件**（`Hardware/`）
| 文件 | 作用 |
|---|---|
| lcd_port.h / .c | 引脚 + SPI2 搬运层，直接用 CubeMX 的 `hspi2` |
| lcd_font.h / .c | 8×16 ASCII（95 字，取自厂家例程）+ 16×16 汉字（我、是） |
| lcd.h / .c | ST7789 初始化、清屏、画点/线/框、ASCII/汉字/UTF-8 字符串 |

**接线（由板上走线决定：原理图 STM32G473VET6.pdf + 厂家例程 .ioc 一致）**

| 信号 | 引脚 | 板上网络 | 说明 |
|---|---|---|---|
| RST | PA5 | EXT_SPI1_SCK | GPIO 输出，低复位 |
| BLK | PA6 | EXT_SPI1_MISO | GPIO 输出，高点亮 |
| CS | PA7 | EXT_SPI1_MOSI | GPIO 输出，常低 |
| DC | PB14 | EXT_SPI2_MISO | GPIO 输出 |
| SDA | PB15 | EXT_SPI2_MOSI | SPI2_MOSI（AF5） |
| SCK | **PB13** | EXT_SPI2_SCK | SPI2_SCK（AF5） |

**接口形式（查证结论）**：屏上没有排针，只有一根 16 pin、0.5 mm 间距的 FFC 软排线，
插到板上的 `FPC1 = FPC-05F-16PH20` 座子（座子 17/18 脚是屏蔽/接地脚，不算信号）。
排线里走的就是：`SCK / SDA / DC / CS / RST / BLK / TE` + `VDD / GND / LEDK`，即 4 线单向 SPI。
屏那一侧是 22 pin 的 `FPC2 = AFC07-S22ECA-00`，中间经转接板接到 16 pin 座子。
所以引脚 100% 由 PCB 铜箔决定、不可自选，也不能接开发板排针。

注：PB14 被当 DC 用，所以 SPI2 必须配成 Half-Duplex Master(1LINE)。
为腾出 PB13，CAN 只保留 FDCAN1（PA11/PA12，单个总线带 2 个 C610 足够），删除 FDCAN2；PB12 变为空闲。
（16 pin 排线的脚号 1→16 顺序在 PDF 里画了两列重复数字，无法唯一判读；但信号集合与 MCU 侧映射已由原理图网络名 + 厂家 .ioc 双向确认。）

**CubeMX 要配的东西**
1. SPI2：Mode = Half-Duplex Master（1LINE），Hardware NSS = Disable（软件 NSS，CS 用 GPIO）
2. 参数：Motorola / 8bit / MSB First / Prescaler 16 / CPOL High / CPHA 2 Edge（≈10.6MHz）/ TI Mode Disable / CRC Disable
3. 引脚：PB13 → SPI2_SCK，PB15 → SPI2_MOSI（都设 Pull-up + High speed；SPI1 不启用，PA5/6/7 只作普通 GPIO）
4. GPIO 输出：PA5、PA6、PA7、PB14（推挽、无上下拉、初始低），User Label 依次 `LCD_RST` `LCD_BLK` `LCD_CS` `LCD_DC`
5. FDCAN2 取消勾选（释放 PB12/PB13）
6. Generate Code（Keep User Code 已勾选，勿取消）

**生成结果核对（2026-10-18，CubeMX 6.17 / FW_G4 1.6.3）**
- SPI2 全部参数与厂家例程一致：Master / 1LINE / 8bit / MSB / CPOL High / CPHA 2Edge / 分频 16 = 10.625 MBit/s，NSS 生成为 `SPI_NSS_SOFT`。
- PA5/PA6/PA7/PB14 = GPIO_OUTPUT_PP（初始低），PB13/PB15 = AF5_SPI2，FDCAN2 已移除。
- `Hardware` 分组与 `../Hardware` 头文件路径**重新生成后仍在**，无需重加。
- Keil 编译 0 Error / 0 Warning，Code=9006 RO=2194 RW=16 ZI=2520。
- 两点遗留（不影响先跑）：① PB13/PB15 生成成 `GPIO_NOPULL` + `SPEED_FREQ_LOW`，厂家例程是 PULLUP + HIGH，10.6MHz 下裕量偏小；② 四个 LCD 脚没打 User Label，`main.h` 无 `LCD_*_Pin` 宏，靠 `lcd_port.h` 的 `#ifndef` 兜底，数值一致。
- **注意**：本次把 FDCAN1 也一起删了，当前工程没有任何 CAN。后面接 M2006/C610 前需重新勾 FDCAN1（PA11/PA12，与 LCD 引脚不冲突）。→ **已重新勾选**，生成了 `Core/Src/fdcan.c`、`Core/Inc/fdcan.h`。
- **已修**：链接报 105 条 `L6406E`（可用存储区全为 0）。本工程有**两条互不相干的构建路径**，要分别填内存区：
  - Keil uVision 走 `MDK-ARM/car_dipan.uvprojx`：`IROM=0x8000000/0x80000`、`IRAM=0x20000000/0x20000`、`OCR_RVCT4`=ROM 值、`OCR_RVCT9`=RAM 值、`Ir2Chk=0`（RVCT4/RVCT9 不能写反）。
  - **EIDE 插件**（Trae 里点 Build 走这条）走 `MDK-ARM/.eide/eide.yml`，**完全不读 uvprojx**：其 `storageLayout` 的 IROM1/IRAM1 原是空串、还多勾了一个空 IROM2，于是生成的 `build/car_dipan/car_dipan.sct` 里全是 `0x0/0x0`。已填 `0x8000000/0x80000`、`0x20000000/0x20000`，IROM2 取消勾选。
  - 两条路径均 0 Error / 0 Warning（Code=9006）。

**代码改动**
- 新增 `Hardware/lcd_port.*`、`lcd_font.*`、`lcd.*`
- `Core/Src/main.c`：仅 USER CODE 段——Includes 加 `#include "lcd.h"`；BEGIN 2 加 `LCD_Init();` 与 `LCD_ShowString(128,112,"我是G473",GREEN,BLACK);`
- `MDK-ARM/car_dipan.uvprojx`：新增 `Hardware` 分组；IncludePath 加 `../Hardware`

**汉字取模**：SimSun 12pt 渲染 16×16，逐行 2 字节、高位在左（已目视校验字符画）。

**已知限制 / 注意**
- CubeMX 重新 Generate 会重写 `.uvprojx`：本次 `Hardware` 分组和 `../Hardware` 路径没丢，但 `IROM1/IRAM1` 被清成 0（见上方“已修”）。**每次 Generate 后都去 Options for Target → Target 页扫一眼存储区。**
- EIDE 的内存区在 `.eide/eide.yml`，Generate 不会动它，但从 Keil 工程导入时会把空值带进来。**改完 eide.yml 要 `Developer: Reload Window` 让插件重读，否则它仍用内存里的旧值。**
- 字库目前只有“我、是”两个汉字，追加汉字需同时改 `hz_code[]` 与 `hz_16x16[]`。

**回滚**
1. 删除 `Hardware/` 整个目录
2. 删掉 `main.c` 中 USER CODE 两处新增
3. `.uvprojx` 删 `Hardware` 分组 + IncludePath 去掉 `../Hardware`

---

## 2026-10-18　上传 GitHub（QU1453/27-car-chassis）

- 仓库：https://github.com/QU1453/27-car-chassis ，**公开**。简介写中文「27电赛小车底盘 / STM32G473VET6 + 2×M2006/C610 + ST7789」。
  **GitHub 仓库名只允许 ASCII**（字母/数字/`-`/`_`/`.`），原名「27电赛小车地盘」建不出来，只能放简介。
- 新增 `.gitignore`：排除 `MDK-ARM/build`、`MDK-ARM/car_dipan`（uVision 输出）、`MDK-ARM/.cmsis`（EIDE 包缓存）、`MDK-ARM/RTE`、`*.o/.d/.crf/.axf/.hex/.map` 等；入库 117 个源文件。
- 本机执行 `git init -b main` → 首个提交 `16a453f` → 推送到 `origin/main` 并设 upstream。
- 环境备注：本机**没有 `gh` CLI**；TRAE 的 GitHub 集成令牌只读（建仓报 403），建仓靠网页手动完成，推送用该令牌走 `http.extraHeader` 认证，令牌不落盘。

---

## 2026-10-18　电机控制分层（BSP / App / Algorithm）+ M2006/C610 手册

**目录结构（新增三层）**

| 目录 | 职责 | 新增文件 |
|---|---|---|
| `BSP/` | 板级外设封装，不含业务 | `bsp_can.c/h`、`bsp_motor.c/h` |
| `Algorithm/` | 纯算法，不依赖硬件 | `algo_pid.c/h`、`algo_kinematics.c/h`（已有 `algo_math.h`） |
| `App/` | 任务与业务逻辑 | `app_config.h`、`app_chassis.c/h` |
| `docs/` | 手册 | `M2006_C610_CAN手册.md` |

**分工**：`App → BSP → Core/HAL`，`App → Algorithm`，Algorithm 不反向依赖。
任务入口统一叫 `xxx_Task()`，在 `main.c` 的 `while(1)` 调用，节拍用 `HAL_GetTick()` 非阻塞实现。

**功能**
- `bsp_can`：全局过滤全收 + 远程帧拒收 + `HAL_FDCAN_Start`（CubeMX 只初始化，不做这两步）；轮询收发。
  `StdFiltersNbr=0` 时自动跳过过滤表配置，靠全局过滤工作。
- `bsp_motor`：0x200/0x1FF 按 ID 分槽大端打包；`0x201~0x208` 解析；多圈角度累加；在线检测。
- `algo_pid`：位置式 PID，积分限幅 + 饱和停止积分。
- `algo_kinematics`：两轮差速正/逆解，输入 m/s、rad/s，输出转子 rpm。
- `app_chassis`：5 ms 速度环（200 Hz），掉线电机给 0 电流。
- `main.c`：仅 USER CODE——Include 加 `app_chassis.h`；BEGIN 2 加 `App_Chassis_Init()` + 示例 `SetVelocity(0.2f, 0.0f)`；BEGIN 3 加 `App_Chassis_Task()`。

**可调参数**：全在 `App/app_config.h`（电机 ID、方向 ±1、轮半径 0.075 m、轮距 0.30 m、PID、限流 3000）。
轮半径/轮距是占位默认值，**需按实车实测修改**。

**eide.yml 修正**
- 补漏文件：`../Core/Src/fdcan.c`、`stm32g4xx_hal_fdcan.c`（原先缺失，EIDE 链接必失败）。
- 新增 `BSP` / `Algorithm` / `App` 虚拟分组与 `../BSP`、`../Algorithm`、`../App` 头文件路径。
- 改完需 `Developer: Reload Window`。

**验证**：用 STM32CubeCLT 的 `arm-none-eabi-gcc -fsyntax-only -Wall` 编译全部新增 .c 与 `main.c`，0 error / 0 warning。EIDE 完整构建待插件内执行。

**⚠ 未改但必须修（CubeMX 生成，禁手改）**
`Core/Src/fdcan.c` 当前 `Prescaler=16 / TimeSeg1=1 / TimeSeg2=1`（≈3.54 Mbps，且 Seg2 寄存器值为非法下限），
`StdFiltersNbr=0`、`AutoRetransmission=DISABLE`、PA11/PA12 `SPEED_FREQ_LOW`。
需在 CubeMX GUI 改为 10 / 13 / 3 / SJW=1、`StdFiltersNbr=1`、`AutoRetransmission=ENABLE`、`VERY_HIGH`。

**回滚**
1. 删 `BSP/`、`App/`、`docs/`，删 `Algorithm/algo_pid.*`、`algo_kinematics.*`
2. `main.c` 删 USER CODE 三处新增
3. `eide.yml` 删新增分组/路径/两个漏文件条目

---

## 2026-10-18　新增 README

- 新建 `README.md`：背景、目的、硬件清单（主控/屏/电机/电调 + 引脚表）、CAN 报文速查、
  分层说明、目录树、工具链、构建步骤、基础规则（禁改 CubeMX 生成文件等）、已知问题。
- 同步重补 `eide.yml`（BSP/Algorithm/App 分组、两个 fdcan 源文件、三个头文件路径）：
  **EIDE 重新导入 Keil 工程时会用 uvprojx 覆盖 eide.yml**，手加内容会丢，每次 Reload 后要检查。

---

## 2026-10-24　底盘动力电机改型：新增 DM-S2325 手册

- 动力电机由 M2006/C610 换成**达妙 DM-S2325-1EC + DM3520-1EC 驱动器**（额定 1.35 Nm / 峰值 5 Nm / 24 V）。
- 新模式选用 **MIT 模式，Kp=0、Kd≠0（定速）**。
- 新建 `docs/DM_S2325_CAN手册.md`：接线、ID 规划、反馈帧、MIT 8 字节位域打包、使能/失能/零点/清错、
  与现有代码差异清单、坑。
- **只写文档，未动任何代码**（`bsp_motor` / `app_config.h` / `algo_kinematics` 仍按 M2006 协议，待硬件到手再改）。
- 新资料解压在 `../_dm_s2325/dm-s2325-1-ec-master/`（只读，未纳入工程）。
- 待确认项：使能/失能/零点/清错的 4 个命令码（`FC/FD/FE/FB`）需用 USB2CAN 实测核对；`P_MAX/V_MAX/T_MAX` 需从电机读参数。
- 新建 `struct.md`：分层依赖、目录树、各层放什么、4 条规矩。

**回滚**：删 `docs/DM_S2325_CAN手册.md`、`struct.md` 即恢复。



## 2026-10-24 DM-S2325 驱动（最简版）
- BSP/bsp_motor.c/h 重写：MIT 帧打包 p16/v12/Kp12/Kd12/t12，反馈解码 pos/vel/tor/温度/ERR，使能失能命令帧；API 改为 Init/SetSpeed/Send/Enable/SendCmd/Decode/IsOnline
- App/app_config.h：新增左右 Master ID(0x11/0x12)；APP_GEAR_RATIO 36→1（DM 量纲已在输出轴）
- App/app_chassis.c/h：去掉外层 PID（DM 内部 Kd 已是速度环），直接下发目标转速；掉线时重发使能
- 回滚：git checkout BSP/bsp_motor.c BSP/bsp_motor.h App/app_chassis.c App/app_chassis.h App/app_config.h

## 2026-10-24 eide.yml 第三次被覆写，补回
- 现象：EIDE 树里没有 BSP/Algorithm/App 三组，bsp_motor.c 不参与编译
- 原因：点「重新导入 Keil 工程」→ 按 car_dipan.uvprojx 重写 eide.yml
- 改：补回 Algorithm(algo_kinematics/algo_pid)、BSP(bsp_can/bsp_motor)、App(app_chassis) 三组；incList 补 ../App ../BSP ../Algorithm

## 2026-10-24 EIDE 分组丢失根治（改 uvprojx + yml，经用户授权）
- `MDK-ARM/car_dipan.uvprojx`：`<Groups>` 末尾加 BSP / Algorithm / App 三个 `<Group>`（5 个 .c）；唯一那处 `<IncludePath>` 前置补 `../App;../BSP;../Algorithm`。只加分组和头文件路径，编译选项一律未动；XML 校验通过（共 8 组）
- `MDK-ARM/.eide/eide.yml`：virtualFolder 加同样三组；incList 加 ../App ../BSP ../Algorithm
- `BSP/bsp_motor.c`、`App/app_chassis.c`：补注释（文件头数据流、MIT 8 字节位域图、反馈帧逐字节布局、各静态变量含义、5 步流程编号）
- 新建 `IMPORTANT_FIXES.md`（重要问题修复记录）
- 5 个 .c 过 `arm-none-eabi-gcc -fsyntax-only -Wall -Wextra` 全 OK
- 回滚：git checkout MDK-ARM/car_dipan.uvprojx MDK-ARM/.eide/eide.yml BSP/bsp_motor.c App/app_chassis.c

## 2026-10-24 Algorithm 层补注释（只加注释，逻辑未改）
- algo_math.h：header-only/static inline 的理由、为什么用 float 不用 double；Signf(0)=0、Deadband 是平移、Map 不夹紧、LowPass 的 alpha、WrapPi 用 while 不用 fmodf
- algo_kinematics.h/c：坐标系约定(+x 车头/+y 左/v 前进为正/w 逆时针为正)、gear_ratio 必须填 1.0f 的理由、逆解 4 步推导 + k=60·gear_ratio/(2πR) 算例、正解除整轮距
- algo_pid.h/c：位置式 vs 增量式、dt 必须真实且固定、anti-windup 只在误差与饱和同向时回滚积分、dt=0 会出 nan 并沿状态传播
- 验证：algo_kinematics.c / algo_pid.c 过 `arm-none-eabi-gcc -fsyntax-only -Wall -Wextra` 无告警
- 回滚：git checkout Algorithm/

## 2026-10-24 注释规范化（文件作用 + 命名对照 + 函数作用）
- 范围：Algorithm 5 个、BSP 4 个、App 3 个、Hardware 6 个，共 18 个自有源文件
- 每个文件头补两段：「文件作用」（一两句说清干什么、给谁用）、「命名对照」（英文标识符 = 中文含义）
- 每个函数开头统一 `/* 作用：... */`，含参数/返回值要点；static 辅助函数同样标注
- Hardware 额外保留端口引脚表与取模规则；lcd_font.c 只有数据，加了取模规则说明
- 只加注释，代码逻辑、宏值、函数签名一字未改
- 验证：8 个 .c 过 `arm-none-eabi-gcc -fsyntax-only -Wall -Wextra` 零告警（ALL-SYNTAX-OK）
- 回滚：git checkout Algorithm/ BSP/ App/ Hardware/

## 2026-10-24 MIT 帧交互演示页
- 新增 docs/mit_frame_demo.html（单文件、零依赖、双击即开）
- 四幕：量化 → 64 位拼位 → 8 字节打包 → 一问一答 + 反馈解码；量程/打包顺序照抄 bsp_motor.h/.c
- 回滚：删除 docs/mit_frame_demo.html

## 2026-11-03 新增 .vscode/tasks.json
- 原因：工程为 Keil MDK，STM32 插件无法直接烧录。
- 内容：新增「编译 / 烧录 / 编译并烧录」三个任务，默认生成任务为「编译并烧录」（Ctrl+Shift+B）。

## 2026-11-03 main.c 中文串编译报错
- 现象：AC5 报 main.c(99) missing closing quote / too few arguments，无法生成 hex。
- 原因：源码 UTF-8，armcc 按 GBK 解析中文字符串，吞掉引号。
- 改法：显示串改为 UTF-8 字节转义字面量（\xE6...），纯 ASCII 源码，LCD 侧仍按 UTF-8 解码。

## 2026-11-07 重新绑定 GitHub 并推送当前版本
- 原因：工程目录迁移，本地 main 与远端分叉（远端多一条引脚配置提交）。
- 改法：HEAD 对齐 origin/main 后提交当前工作区全部内容并推送；.gitignore 补 build.log / flash_build.log；仓库描述改为「电赛小车地盘」。

## 2026-11-08 修复合并提交里的冲突标记
- 现象：合并提交 b57fa15 把 `<<<<<<< / ======= / >>>>>>>` 标记一起提交进 `Hardware/lcd_font.c`，Keil 编译报语法错误。
- 改动：删除标记，保留点阵数据；清掉 `///////111111111` 垃圾注释。其余文件无标记。
- 回滚：`git revert <本次提交>`
