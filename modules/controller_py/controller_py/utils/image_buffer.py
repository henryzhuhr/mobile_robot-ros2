import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import Image

from interfaces.msg import (Lanes, ImageViewer)
import cv2


# class LanesXY:

#     COLOR_MAP = {
#         "yellow": (255, 255, 0),     # 0 左左车道线
#         "blue": (0, 0, 255),         # 1 左车道线
#         "green": (0, 255, 0),        # 2 右车道线
#         "red": (255, 0, 0),          # 3 右右车道线
#         #
#         "darkyellow": (255, 165, 0),  # 0 左左车道线 的卡尔曼滤波后的车道线
#         "darkblue": (0, 0, 139),     # 1 左车道线 的卡尔曼滤波后的车道线
#         "darkgreen": (0, 100, 0),    # 2 右车道线 的卡尔曼滤波后的车道线
#         "darkred": (139, 0, 0),      # 3 右右车道线 的卡尔曼滤波后的车道线
#         #
#         "magenta": (255, 0, 255),    # 中心车道线
#         "white": (255, 255, 255),    # 车道线
#         "black": (0, 0, 0),          # 背景
#         "darkblue": (139, 0, 0),
#         "darkgreen": (0, 139, 0),
#         "red": (255, 0, 0),
#         "magenta": (255, 0, 255),
#         "white": (255, 255, 255),
#         "black": (0, 0, 0),
#         "gray": (128, 128, 128),
#         "orange": (255, 165, 0),
#     }
#     COLOR_LIST = [(b, g, r) for (color_name, (r, g, b)) in COLOR_MAP.items()]

#     def __init__(self, lanes: Lanes = None) -> None:

#     @staticmethod
#     def msg_lanes_to_xy(msg: Lanes):
#         return (
#             np.array(msg.lanes_x_coords).reshape(4, 18),
#             np.array(msg.lanes_y_coords).reshape(18),
#         )


class LinkedNode:
    def __init__(self, data: ImageViewer):
        self.data = data
        self.next: LinkedNode = None


class ImageViewerBuffer:
    def __init__(self, maxsize: int = -1,  # -1 means no limit
                 ) -> None:
        self.__head = LinkedNode(None)
        self.__tail = self.__head
        self.__size = 0
        self.maxsize = maxsize

    def size(self) -> int:
        return self.__size

    def append(self, image_viewer_msg: ImageViewer):
        node = LinkedNode(image_viewer_msg)
        self.__tail.next = node
        self.__tail = node
        self.__size += 1
        if self.__size > self.maxsize:
            self.pop()

    def pop(self):
        if self.__head.next is None:
            return None
        node: LinkedNode = self.__head.next
        self.__head.next = node.next
        self.__size -= 1
        if self.__size == 0:
            self.__tail = self.__head
        return node.data

    def clear(self):
        self.__head = LinkedNode(None)
        self.__tail = self.__head
        self.__size = 0

    def find_header_and_set(self, data: ImageViewer)->int :
        p = self.__head
        while p.next is not None:
            p = p.next
            if p.data.header.stamp == data.header.stamp:
                if data.lanes_x_coords is not None and data.lanes_y_coords is not None:
                    p.data.flag |= 2
                    p.data.lanes_x_coords = data.lanes_x_coords
                    p.data.lanes_y_coords = data.lanes_y_coords
                return 2
        return -1
