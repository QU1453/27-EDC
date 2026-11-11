# 电赛视觉主程序（MaixCAM2 + MaixPy）
# 流程：720P 采集 -> 320x240 识别 -> 在原图上标注 -> UART4 发送目标坐标
# 协议：每帧一行文本 "cx,cy,area\n"，未找到目标时发 "-1,-1,0\n"
# 注意：串口坐标基于 320x240 识别分辨率，不是原图

from maix import camera, display, image, app, time

from uart_link import UartLink
from detect import find_target, THRESHOLDS

CAP_W, CAP_H = 1280, 720    # 采集分辨率；RGB888 上限 1920x1080
DET_W, DET_H = 320, 240     # 识别分辨率，越小越快
MIN_AREA = 300              # 色块最小面积（按 320x240 计），用于滤除噪点
CAM_FPS = 60                # 采集帧率；传感器不支持时会退化成默认值
PREVIEW = True              # False = 不显示，比赛模式可省掉预览开销
PREVIEW_EVERY = 1           # 每 N 帧显示一次，2 表示隔帧显示

# 识别坐标 -> 原图坐标的缩放系数
SCALE_X = CAP_W / DET_W
SCALE_Y = CAP_H / DET_H

cam = camera.Camera(CAP_W, CAP_H, fps=CAM_FPS, buff_num=3)
try:
    # 第二路直接输出小图给识别，省掉每帧 resize
    cam_det = cam.add_channel(DET_W, DET_H, fps=CAM_FPS)
except Exception as e:
    print("add_channel 不可用，改用 resize:", e)
    cam_det = None

disp = display.Display()
link = UartLink()

fps_t = time.ticks_ms()
fps_cnt = 0
frame_cnt = 0

while not app.need_exit():
    if cam_det is not None:
        small = cam_det.read()                    # 识别通道：直接出小图
        frame = cam.read() if PREVIEW else None   # 预览通道：不显示就不读，省带宽
    else:
        frame = cam.read()
        small = frame.resize(DET_W, DET_H)

    target = find_target(small, THRESHOLDS, MIN_AREA)
    if target is None:
        # 丢目标：STM32 侧据此判断进入搜索/停车逻辑
        link.send(-1, -1, 0)
    else:
        cx, cy, area, (x, y, w, h) = target
        link.send(cx, cy, area)

        if frame is not None:
            # 在 720P 原图上标注，坐标乘以缩放系数
            X, Y = int(x * SCALE_X), int(y * SCALE_Y)
            W, H = int(w * SCALE_X), int(h * SCALE_Y)
            frame.draw_rect(X, Y, W, H, image.COLOR_GREEN, thickness=3)
            frame.draw_cross(int(cx * SCALE_X), int(cy * SCALE_Y),
                             image.COLOR_RED, size=20, thickness=3)

    # 每秒打印一次帧率，方便评估算法耗时
    fps_cnt += 1
    if time.ticks_ms() - fps_t >= 1000:
        print(f"FPS: {fps_cnt}")
        fps_cnt = 0
        fps_t = time.ticks_ms()

    # 显示原图（预览清晰但传输慢）；想提高预览帧率改成 disp.show(small)
    if PREVIEW:
        frame_cnt += 1
        if frame_cnt % PREVIEW_EVERY == 0:
            disp.show(frame)
