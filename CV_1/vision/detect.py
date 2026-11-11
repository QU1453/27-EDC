# 传统视觉算法：LAB 颜色阈值找色块
# 阈值格式：[l_min, l_max, a_min, a_max, b_min, b_max]
# 快速获取方法：用设备自带“找色块”应用，点选目标物体后把显示的阈值抄过来

THRESHOLDS = [
    [0, 80, 40, 80, 10, 80],       # 红色
]

# 备用：黑色巡线阈值，按现场光照微调
LINE_THRESHOLDS = [
    [0, 40, -20, 20, -20, 20],
]


def find_target(img, thresholds, min_area=300):
    """找最大色块，返回 (中心x, 中心y, 面积, (x, y, w, h))；未找到返回 None。
    坐标和面积都基于传入 img 的分辨率。
    """
    blobs = img.find_blobs(thresholds,
                           area_threshold=min_area,
                           pixels_threshold=min_area)
    if not blobs:
        return None

    # blob 可下标访问：blob[0]=x, blob[1]=y, blob[2]=w, blob[3]=h
    b = max(blobs, key=lambda item: item[2] * item[3])
    x, y, w, h = b[0], b[1], b[2], b[3]

    cx = x + w // 2
    cy = y + h // 2
    return cx, cy, w * h, (x, y, w, h)
