from os import name
from typing import Any, Dict, List
import cv2
import numpy as np

from .constants import (
    GRIDING_NUM,
    CLS_NUM_PER_LANE,
    ROW_ANCHOR,
    COLOR_LIST,
    COLOR_MAP,
)

from .process import (
    img2tensor,
    phrasing_output,
)

from .kalman import LaneKalmanFiter


class InferResult:
    """模型推理结果
    - `lanes_y_coords`:         `np.ndarray([18], dtype=np.int32)`      所有车道线共用一组 y 坐标
    - `lanes_x_coords`:         `np.ndarray([4, 18], dtype=np.int32)`   4 个车道线的 x 坐标
    - `lanes_x_coords_kl`:      `np.ndarray([4, 18], dtype=np.int32)`   卡尔曼滤波后 4 个车道线的 x 坐标
    - `lane_center_x_coords`:   `np.ndarray([18], dtype=np.int32)`      中心车道线的 x 坐标
    - `forward_direct`:         `np.ndarray([2, 2], dtype=np.int32)`    实际前进方向
    - `predict_direct`:         `np.ndarray([2, 2], dtype=np.int32)`    预测前进方向
    - `slope`:                  `np.float64`    预测方向的斜率
    - `offset_distance`:        `np.int32`      偏移距离

    其中前进方向定义为 最底部的点`(x0,y0)`和上一个点`(x1,y1)`的连线
    """
    def __init__(self) -> None:
        self.lanes_y_coords = np.zeros([18], dtype=np.int32)
        self.lanes_x_coords = np.zeros([4, 18], dtype=np.int32)
        self.lanes_x_coords_kl = np.zeros([4, 18], dtype=np.int32)
        self.lane_center_x_coords = np.zeros([18], dtype=np.int32)
        self.forward_direct = np.zeros([2, 2], dtype=np.int32)
        self.predict_direct = np.zeros([2, 2], dtype=np.int32)
        self.slope: np.float64 = np.float64(0)
        self.offset_distance: np.int32 = np.int32(0)


class BaseModelInfer:
    """
    基本模型推理类

    继承此类，实现 `infer_model()` 方法对模型进行推理即可，
    输入输出均为 `np.ndarray` 且为  `np.float32` 类型，数值范围: `[0, 1]`
    输入输出形状如下：  
    - `input` : `(1, C, H, W)`
    - `output`: `(201, 18, 4)`

    ```python
    def infer_model(self, input: np.ndarray) -> np.ndarray:
        '''
        # 执行模型推理，子类必须实现此方法
        ## Args:
        - `input`: `np.ndarray`, `(1, C, H, W)`, `np.float32`, range: `[0, 1]`
        
        ## Returns:
        - `output`: `np.ndarray`, `(201, 18, 4)`, `np.float32`, range: `[0, 1]`
        '''
        raise NotImplementedError
    ```
    """
    def __init__(
        self,
        griding_num=GRIDING_NUM,
        cls_num_per_lane=CLS_NUM_PER_LANE,
        row_anchor=ROW_ANCHOR,
        klf_id=[                           # 真正执行卡尔曼滤波的车道线 id
            0,                             # 0 左左车道线
            1,                             # 1 左车道线
            2,                             # 2 右车道线
            3,                             # 3 右右车道线
        ]
    ) -> None:

        self.griding_num = griding_num
        self.cls_num_per_lane = cls_num_per_lane
        self.row_anchor = row_anchor
        self.color_list = COLOR_LIST
        self.color_map = COLOR_MAP
        self.klf_id = klf_id

        # 加载 Kalman，4 个 LKF 必须是不同的对象
        self.lanes_klf = [LaneKalmanFiter() for _ in range(4)]

    def infer_model(self, input: np.ndarray) -> np.ndarray:
        """
        # 执行模型推理，子类必须实现此方法
        ## Args:
        - `input`: `np.ndarray`, `(1, C, H, W)`, `np.float32`, range: `[0, 1]`
        
        ## Returns:
        - `output`: `np.ndarray`, `(201, 18, 4)`, `np.float32`, range: `[0, 1]`
        """
        raise NotImplementedError

    def infer(self, img: np.ndarray):
        """
        ### Args:
        - `img`: `np.ndarray`(`np.uint8`), 形状 `(H, W, C)`, 数值范围: `[0, 255]`
        ### Returns:
        - `infer_result`: `InferResult`
        """
        img_h, img_w, _ = img.shape
        input = cv2.resize(img, dsize=(800, 288))
        input = img2tensor(input)

        # 推理模型
        model_out = self.infer_model(input)

        # 解析模型输出
        (
            lanes_x_coords,        # [4,18] 4 个车道线 x 坐标
            lane_y_coords,         # [18]   所有车道线共用一组 y 坐标
        ) = phrasing_output(
            model_out,
            img_h,
            img_w,
            self.griding_num,
            self.cls_num_per_lane,
            self.row_anchor,
        )
        """
        (720, 1280, 3)
        [716 691 666 644 619 594 569 546 521 496 471 449 424 399 374 351 326 301]
        """

        # 卡尔曼滤波
        lanes_x_coords_kl = np.zeros_like(lanes_x_coords, dtype=np.int32)
        for i in range(4):
            if i in self.klf_id:
                lanes_x_coords_kl[i] = self.lanes_klf[i].update(lanes_x_coords[i], lane_y_coords)

        # 计算中心线的 x 坐标
        lane_center_x_coords = BaseModelInfer.compute_center_lane(
            lanes_x_coords_kl[1], lanes_x_coords_kl[2], lane_y_coords
        )

        # 计算预测方向的斜率
        (
            slope,    # 斜率
            (x0, y0), # 中心车道线，最底部的点
            (x1, y1), # 中心车道线，最底部的上一个点
        ) = BaseModelInfer.compute_slope(lane_center_x_coords, lane_y_coords)

        # 实际的前进方向
        # TODO ，需要实际标定，目前使用图像中心线作为前进方向
        forward_direct = np.array(((img_w // 2, img_h), (img_w // 2, img_h / 5)), dtype=np.int32)
        # 预测的前进方向
        predict_direct = np.array(((x0, y0), (x1, y1)), dtype=np.int32)

        # TODO: 预测方向于左右的距离，需要实际标定

        # 计算偏移距离，取最底部的点的 x 坐标计算，用像素表示，(以图像中心为原点，向右为正方向)，
        if x0 == 0:
            offset_distance = np.int32(0)
        else:
            offset_distance = np.int32(predict_direct[0][0] - forward_direct[0][0])

        infer_result = InferResult()
        infer_result.lanes_y_coords = lane_y_coords
        infer_result.lanes_x_coords = lanes_x_coords
        infer_result.lanes_x_coords_kl = lanes_x_coords_kl
        infer_result.lane_center_x_coords = lane_center_x_coords
        infer_result.forward_direct = forward_direct
        infer_result.predict_direct = predict_direct
        infer_result.slope = slope
        infer_result.offset_distance = offset_distance
        return infer_result

    @staticmethod
    def compute_center_lane(left__lane_x_coords, right_lane_x_coords, lane_y_coords):
        """
        计算中心车道线
        """
        lane_center_x_coords = np.zeros(left__lane_x_coords.shape, dtype=np.int32)
        for i in range(lane_y_coords.shape[0]):
            left__x = left__lane_x_coords[i]
            right_x = right_lane_x_coords[i]
            y = lane_y_coords[i]

            # 如果左右车道线有一个不存在，则中心车道线也不存在
            if left__x == 0 or right_x == 0:
                lane_center_x_coords[i] = 0
            else:
                lane_center_x_coords[i] = int((left__x + right_x) / 2)
        return lane_center_x_coords

    @staticmethod
    def compute_slope(lane_x_coords, lane_y_coords):
        """
        计算预测方向的斜率

        ### Args:
        - `lane_x_coords`: `np.ndarray`, shape: `[18]`, dtype: `np.int32`
        - `lane_y_coords`: `np.ndarray`, shape: `[18]`, dtype: `np.int32`   
        ### Returns:
        如果无法拟合车道线，则返回 `(0, (0, 0), (0, 0))`
        - `slope`: `float`
        - `p0`: `(x0, y0)`, `int32`, 中心车道线，最底部的点
        - `p1`: `(x1, y1)`, `int32`, 中心车道线，最底部的上一个点
        """

        # 找到非零的点，记录下索引
        valid_point_index = []
        for i in range(lane_y_coords.shape[0]):
            x = lane_x_coords[i]
            if x != 0:
                valid_point_index.append(i)

        if len(valid_point_index) < 2:
            return (np.float64(0), (0, 0), (0, 0))
        else:
            # 从有效点中提取 x 和 y 坐标
            x = np.array([lane_x_coords[i] for i in valid_point_index], dtype=np.int32)
            y = np.array([lane_y_coords[i] for i in valid_point_index], dtype=np.int32)

            # 根据有效点拟合二次曲线
            a, b, c = np.polyfit(y, x, 2) # TODO: RankWarning: Polyfit may be poorly conditioned

            y0 = y[0]
            y1 = y[1]

            x0 = a * y0 * y0 + b * y0 + c
            x1 = a * y1 * y1 + b * y1 + c
            """
            cv2.Mat 的 x,y 坐标系
            .-----------------------------> x
            |
            |              ·  p1(x1,y1)
            |            /
            |           /
            |          /
            |         / 
            |        · p0(x0,y0)
            y
            由于 y0>y1 所以计算斜率的公式为: slope = (y0 - y1) / (x1 - x0)
            """
            if x0 == x1:
                slope = np.float64(0)
            else:

                slope = np.float64((y0 - y1) / (x1 - x0))
            return (slope, (int(x0), y0), (int(x1), y1))

    @staticmethod
    def mark_lane(img: np.ndarray, lane_x_coords: np.ndarray, lane_y_coords: np.ndarray, color: tuple = (0, 0, 255)):
        for i in range(lane_y_coords.shape[0]):
            x = int(lane_x_coords[i])
            y = int(lane_y_coords[i])
            if x != 0:
                cv2.circle(img, (x, y), 5, color, -1)

    @staticmethod
    def mark_result(img: np.ndarray, infer_result: InferResult):

        lane_y_coords = infer_result.lanes_y_coords              # [18]     所有车道线共用一组 y 坐标
        lanes_x_coords = infer_result.lanes_x_coords             # [4, 18]  4 个车道线的 x 坐标
        lanes_x_coords_kl = infer_result.lanes_x_coords_kl       # [4, 18]  卡尔曼滤波后 4 个车道线的 x 坐标
        lane_center_x_coords = infer_result.lane_center_x_coords # [18]     中心车道线的 x 坐标
        forward_direct = infer_result.forward_direct             # [2, 2]   实际前进方向
        predict_direct = infer_result.predict_direct             # [2, 2]   预测前进方向
        slope = infer_result.slope                               # 预测方向的斜率
        offset_distance = infer_result.offset_distance           # 偏移距离

        img_h, img_w, _ = img.shape

        lane_num = lanes_x_coords.shape[0]
        for i in range(lane_num):
            color = COLOR_LIST[i]
            BaseModelInfer.mark_lane(img, lanes_x_coords[i], lane_y_coords, color)
            color = COLOR_LIST[i + lane_num]
            BaseModelInfer.mark_lane(img, lanes_x_coords_kl[i], lane_y_coords, color)

        for i in range(lane_num):
            color = COLOR_LIST[i + lane_num]

            non_zero_index = np.nonzero(lanes_x_coords_kl[i])# 找到非零的点，记录下索引

            non_zero_x_coords = lanes_x_coords_kl[i][non_zero_index]
            non_zero_y_coords = lane_y_coords[non_zero_index]

            for j in range(non_zero_x_coords.shape[0] - 1):
                p0 = (non_zero_x_coords[j], non_zero_y_coords[j])
                p1 = (non_zero_x_coords[j + 1], non_zero_y_coords[j + 1])
                cv2.line(img, p0, p1, color, 2)

        BaseModelInfer.mark_lane(img, lane_center_x_coords, lane_y_coords, COLOR_LIST[lane_num + lane_num + 1])
        cv2.line(img, tuple(forward_direct[0]), tuple(forward_direct[1]), COLOR_LIST[lane_num * 2 + 2], 2)

        # 绘制预测方向(延长)
        if predict_direct[0][1] != 0 and predict_direct[1][1] != 0:
            a, b = np.polyfit(predict_direct[:, 1], predict_direct[:, 0], 1)
            y0 = forward_direct[0][1]
            y1 = forward_direct[1][1]
            predict_direct = np.array(((int(a * y0 + b), y0), (int(a * y1 + b), y1)), dtype=np.int32)
        cv2.line(img, tuple(predict_direct[0]), tuple(predict_direct[1]), COLOR_LIST[lane_num * 2 + 3], 2)

        # 在图像中输出图像尺寸，偏移距离，预测方向的斜率
        color = (255, 255, 255)
        for line, text in enumerate(
            [
                f"size  : {img_w},{img_h}",
                f"offset: {offset_distance}",
                f"slope : {slope:.2f}",
            ]
        ):
            # 按行显示，需要根据图像高度调整
            show_line_num = 20 # 总共可以显示的行数，可以修改
            base_scale = 30    # 这个数值越大，字体越小

            line_height = img_h / show_line_num
            scale = line_height / base_scale
            thickness = int(scale * 2)

            cv2.putText(
                img,
                text,
                org=(int(0.01 * img_h), int((line + 1) * line_height)),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=scale,
                color=color,
                thickness=thickness,
            )
