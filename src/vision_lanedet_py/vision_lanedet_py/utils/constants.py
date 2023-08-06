# ===============================================
#  Hyper Parameters for UFLD + CULane Dataset
# ===============================================

CLS_NUM_PER_LANE = 18
GRIDING_NUM = 200  # same as `configs/culane.py`
ROW_ANCHOR = [121, 131, 141, 150, 160, 170, 180, 189, 199, 209, 219, 228, 238, 248, 258, 267, 277, 287]

COLOR_MAP = {
    "yellow": (255, 255, 0),     # 0 左左车道线
    "blue": (0, 0, 255),         # 1 左车道线
    "green": (0, 255, 0),        # 2 右车道线
    "red": (255, 0, 0),          # 3 右右车道线
                                   #
    "darkyellow": (255, 165, 0), # 0 左左车道线 的卡尔曼滤波后的车道线
    "darkblue": (0, 0, 139),     # 1 左车道线 的卡尔曼滤波后的车道线
    "darkgreen": (0, 100, 0),    # 2 右车道线 的卡尔曼滤波后的车道线
    "darkred": (139, 0, 0),      # 3 右右车道线 的卡尔曼滤波后的车道线
                                   #
    "magenta": (255, 0, 255),    # 中心车道线
    "white": (255, 255, 255),    # 车道线
    "black": (0, 0, 0),          # 背景
    "darkblue": (139, 0, 0),
    "darkgreen": (0, 139, 0),
    "red": (255, 0, 0),
    "magenta": (255, 0, 255),
    "white": (255, 255, 255),
    "black": (0, 0, 0),
    "gray": (128, 128, 128),
    "orange": (255, 165, 0),
}
COLOR_LIST = [(b, g, r) for (color_name, (r, g, b)) in COLOR_MAP.items()]
