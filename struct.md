# 项目文件结构

更新时间：2026-11-11

## Git
- 远端：`https://github.com/QU1453/27-EDC.git`（原 27-car-chassis，GitHub 自动重定向）
- 仓库根 = `27电赛/`，CubeMX 文件远端路径为 `Car_Chassis/car_dipan.ioc`
- 分支：`main`（主线）、`QU1453`、`Juneee`

```
27电赛/
├── Car_Chassis/              # STM32G473 底盘工程（CubeMX 生成，勿改生成文件）
├── CV_1/
│   ├── vision/               # MaixCAM2 传统视觉工程（MaixPy，MaixVision 打开此目录）
│   │   ├── main.py           # 入口：取图 → 找色块 → UART 发送
│   │   ├── detect.py         # 传统视觉算法与 LAB 阈值
│   │   ├── uart_link.py      # UART4 封装（A21/A22, /dev/ttyS4, 115200）
│   │   └── app.yaml          # 打包应用信息（id/name/version）
│   └── vision_yolo/          # MaixCAM2 YOLO 工程（独立运行，模型需放设备 /root/models/）
│       ├── main.py           # 入口：采集 → YOLO11 推理 → UART 发送
│       ├── uart_link.py      # UART4 封装（与 vision 相同，独立副本）
│       └── app.yaml          # 打包应用信息（id: race_vision_yolo）
├── .gitignore                # 根级忽略：.cache/、__pycache__/、*.pyc
├── .cache/                   # pip/conda 临时文件（勿提交）
├── modify.md                 # 修改记录
└── struct.md                 # 本文件
```
