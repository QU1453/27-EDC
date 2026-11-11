# 项目文件结构

更新时间：2026-11-06

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
├── .cache/                   # pip/conda 临时文件（勿提交）
├── modify.md                 # 修改记录
└── struct.md                 # 本文件
```
