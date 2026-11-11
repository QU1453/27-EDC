# YOLO 目标检测主程序（MaixCAM2 + MaixPy）
# 流程：按模型输入尺寸采集 -> YOLO11 推理 -> 选目标 -> UART4 发送中心坐标
# 协议：每帧一行 "cx,cy,area\n"，未检测到目标发 "-1,-1,0\n"
# 坐标基准是模型输入尺寸（如 640x640），画面中心为 W//2, H//2
# 使用前把 .mud 和 .cvimodel 放到设备 /root/models/ 目录

from maix import camera, display, image, app, time, nn

from uart_link import UartLink

MODEL_PATH = "/root/models/yolo11n.mud"   # 模型描述文件路径（设备上）

TARGET_CLASS = None    # None = 所有类别；填类别名（如 "ball"）则只跟踪该类
CONF_TH = 0.5          # 置信度阈值，误检多就调高
IOU_TH = 0.45          # NMS 交并比阈值
SEND_LARGEST = True    # True 取面积最大目标，False 取置信度最高目标

model = nn.YOLO11(model=MODEL_PATH, dual_buff=True)
W, H = model.input_width(), model.input_height()

cam = camera.Camera(W, H)   # 采集分辨率直接等于模型输入，省一次缩放
disp = display.Display()
link = UartLink()

fps_t = time.ticks_ms()
fps_cnt = 0

while not app.need_exit():
    img = cam.read()
    objs = model.detect(img, conf_th=CONF_TH, iou_th=IOU_TH)

    # 按类别名过滤（TARGET_CLASS 为 None 时不过滤）
    if TARGET_CLASS is not None:
        objs = [o for o in objs if model.labels[o.class_id] == TARGET_CLASS]

    if not objs:
        # 丢目标：STM32 侧据此进入搜索/停车逻辑
        link.send(-1, -1, 0)
    else:
        if SEND_LARGEST:
            target = max(objs, key=lambda o: o.w * o.h)
        else:
            target = max(objs, key=lambda o: o.score)

        cx = int(target.x + target.w / 2)
        cy = int(target.y + target.h / 2)
        link.send(cx, cy, int(target.w * target.h))

        # 选中的目标画绿框，其余画红框，方便现场确认跟踪对象
        for o in objs:
            color = image.COLOR_GREEN if o is target else image.COLOR_RED
            img.draw_rect(o.x, o.y, o.w, o.h, color, thickness=2)
            img.draw_string(o.x, o.y - 12,
                            f"{model.labels[o.class_id]} {o.score:.2f}",
                            color, scale=1.2)

    # 每秒打印一次帧率
    fps_cnt += 1
    if time.ticks_ms() - fps_t >= 1000:
        print(f"FPS: {fps_cnt}")
        fps_cnt = 0
        fps_t = time.ticks_ms()

    disp.show(img)
