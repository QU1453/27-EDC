# 修改记录

## 2026-11-13 合并本地 CubeMX 外设配置到 Car_Chassis
- 来源：本地分支 `backup/local-20261113`（桌面 car 工程），已适配远端新目录 `Car_Chassis/`。
- `Core/Inc`、`Core/Src`（fdcan / usart / gpio / stm32g4xx_it）：新增 FDCAN2、USART2、USART3、UART4、UART5 的初始化与中断。
- `car_dipan.ioc`：同步上述外设与引脚配置。
- `main.c`：增加 5 个 Init 调用，保留底盘 App 初始化。
- 未改动 `MDK-ARM/`，保留远端 EIDE/Keil 工程配置。

## 2026-11-06 新建视觉工程骨架
- 新增 `CV_1/vision/`：`main.py`（找色块+串口输出）、`detect.py`（阈值/算法）、`uart_link.py`（UART4 封装）、`app.yaml`。
- 协议：`cx,cy,area\n`，丢失目标发 `-1,-1,0\n`；识别分辨率 320x240。

## 2026-11-06 新建 PC 端补全环境
- conda 环境 `maixpy`（Python 3.10.22，路径 `D:\...\anaconda\envs\maixpy`），装 MaixPy 4.12.1、numpy 2.2.6、opencv 5.0.0。
- 仅用于 MaixVision 代码补全，代码实际在设备上运行。

## 2026-11-06 采集分辨率改为 2K
- `main.py`：采集 2560x1440，缩小到 320x240 识别，标注时坐标乘缩放系数；串口坐标仍按 320x240。
- `detect.py`：`find_target` 不再内部画框，改为返回 `(cx, cy, area, (x,y,w,h))`。

## 2026-11-06 采集分辨率回退到 1080P
- 原因：RGB888 格式限制最大 1920x1080，2560x1440 初始化报 RuntimeError。
- 改法：`CAP_W, CAP_H = 1920, 1080`，其余流水线不变。

## 2026-11-06 采集分辨率定为 720P
- 原因：1080P 帧率偏低，720P 可稳定约 30 帧，对电赛足够。
- 改法：`CAP_W, CAP_H = 1280, 720`，识别仍 320x240，串口协议不变。

## 2026-11-06 新建 YOLO 工程
- 新增 `CV_1/vision_yolo/`：`main.py`（YOLO11 推理+串口）、`uart_link.py`、`app.yaml`（id: race_vision_yolo）。
- 坐标基准改为模型输入尺寸（如 640x640），中心为 W/2,H/2；协议仍为 `cx,cy,area\n`。
- 模型文件需放在设备 `/root/models/`，路径由 `MODEL_PATH` 指定。

## 2026-11-11 仓库升级为整个电赛工程
- 远端仓库更名为 `27-EDC`；git 根目录上移到 `27电赛/`，原路径整体变为 `Car_Chassis/...`（`car_dipan.ioc` 与远端最新 blob 一致，未改动）。
- 新增根 `.gitignore`：排除 `.cache/`、`__pycache__/`、`*.pyc`。
- 推送 `main`，并新建并推送 `QU1453`、`Juneee` 两个分支。

## 2026-11-06 vision 性能优化
- 采集加 `fps=60, buff_num=3`；新增第二路 320x240 通道做识别，省 resize。
- 新增 `PREVIEW`、`PREVIEW_EVERY`；关闭预览时不再读 720P 主通道。
- `add_channel` 失败自动退回 resize 方案。
