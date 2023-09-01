from typing import Tuple
import cv2
import numpy as np
from .constants import (
    GRIDING_NUM,
    CLS_NUM_PER_LANE,
    ROW_ANCHOR,
    COLOR_LIST,
    COLOR_MAP,
    CAR_WIDTH_RATIO,
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
    - `y_offset`:               `np.float64`    y 方向上偏移
    - `z_offset`:               `np.int32`      z 方向角偏移

    其中前进方向定义为 最底部的点`(x0,y0)`和上一个点`(x1,y1)`的连线
    """
    def __init__(self) -> None:
        self.lanes_y_coords = np.zeros([18], dtype=np.int32)
        self.lanes_x_coords = np.zeros([4, 18], dtype=np.int32)
        self.lanes_x_coords_kl = np.zeros([4, 18], dtype=np.int32)
        self.lane_center_x_coords = np.zeros([18], dtype=np.int32)
        self.forward_direct = np.zeros([2, 2], dtype=np.int32)
        self.predict_direct = np.zeros([2, 2], dtype=np.int32)
        self.y_offset: np.float64 = np.float64(0)
        self.z_offset: np.int32 = np.int32(0)


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
        car_width_ratio: float = CAR_WIDTH_RATIO, # 车宽度 (占宽度百分比)
        griding_num=GRIDING_NUM,
        cls_num_per_lane=CLS_NUM_PER_LANE,
        row_anchor=ROW_ANCHOR,
        klf_id=[                                            # 真正执行卡尔曼滤波的车道线 id
            0,                                              # 1 左左车道线
            1,                                              # 2 左车道线
            2,                                              # 3 右车道线
            3,                                              # 4 右右车道线
        ]
    ) -> None:

        self.car_width_ratio = car_width_ratio # 车宽度 (占宽度百分比)
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

    def infer(self, img: np.ndarray, is_kalman=False):
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
        """
        (720, 1280, 3)
        [716 691 666 644 619 594 569 546 521 496 471 449 424 399 374 351 326 301]
        """
        (
            lanes_x_coords, # np.int32 [4,18] 4 个车道线 x 坐标
            lane_y_coords,  # np.int32 [18]   所有车道线共用一组 y 坐标
        ) = self.phrasing_output(model_out, img_h, img_w, self.griding_num, self.cls_num_per_lane, self.row_anchor)

        # 拟合左右车道线并且重新采样
        (
            left__lane_x,                # 重新拟合左车道线
            right_lane_x,                # 重新拟合右车道线
        ) = self.fit_and_resample_lanes(
            lanes_x_coords[1],
            lanes_x_coords[2],
            lane_y_coords,
        )
        lanes_x_coords[1] = left__lane_x
        lanes_x_coords[2] = right_lane_x

        # lanes_x_coords[1] = np.zeros_like(left__lane_x, dtype=np.int32)
        # lanes_x_coords[2] = np.zeros_like(right_lane_x, dtype=np.int32)

        lanes_x_coords_kl = np.zeros_like(lanes_x_coords, dtype=np.int32)  # [4,18]


        # 卡尔曼滤波
        if is_kalman:
            for i in self.klf_id:
                lanes_x_coords_kl[i] = self.lanes_klf[i].update(lanes_x_coords[i], lane_y_coords)

        # 计算中心线的 x 坐标
        lane_center_x_coords = self.compute_center_lane(
            lanes_x_coords_kl[1] if is_kalman else lanes_x_coords[1],
            lanes_x_coords_kl[2] if is_kalman else lanes_x_coords[2],
            lane_y_coords,
        )

        # 计算预测方向的斜率
        (
            left_slope,    # 斜率 0.75
            (left_x0, left_y0), # 中心车道线，最底部的点
            (left_x1, left_y1), # 中心车道线，最底部的上一个点
        ) = BaseModelInfer.compute_slope(lanes_x_coords[1], lane_y_coords)

        # 计算预测方向的斜率
        (
            right_slope,    # 斜率 -0.7
            (right_x0, right_y0), # 中心车道线，最底部的点
            (right_x1, right_y1), # 中心车道线，最底部的上一个点
        ) = BaseModelInfer.compute_slope(lanes_x_coords[2], lane_y_coords)

        # 计算预测方向的斜率
        (
            slope,    # 斜率
            (x0, y0), # 中心车道线，最底部的点
            (x1, y1), # 中心车道线，最底部的上一个点
        ) = BaseModelInfer.compute_slope(lane_center_x_coords, lane_y_coords)
        # print("slope", left_slope, right_slope, slope)
        # print("left_x0", left_x0)
        # print("right_x0", right_x0)
        # 计算方向角偏移
        if (left_slope == 0 and right_slope != 0) :
            z_offset = np.arctan(right_slope + 0.75)
        elif (left_slope != 0 and right_slope == 0) :
            z_offset = np.arctan(left_slope - 0.8)
        else:
            z_offset = np.arctan(slope)

        # 实际的前进方向
        # TODO ，需要实际标定，目前使用图像中心线作为前进方向
        # w:1280  [148 (594) 1040]
        forward_direct = np.array(((img_w // 2, img_h), (img_w // 2, img_h / 5)), dtype=np.int32)
        # 预测的前进方向
        predict_direct = np.array(((x0, y0), (x1, y1)), dtype=np.int32)

        # TODO: 预测方向于左右的距离，需要实际标定

        # 计算偏移距离，取最底部的点的 x 坐标计算，用像素表示，(以图像中心为原点，向右为正方向)，
        # 对应到小车就是 y 轴方向
        if (left_x0 == 0 and right_x0 == 0):
            y_offset = np.int32(0)
        elif (left_x0 != 0 and right_x0 == 0):
            y_offset = np.int32(left_x0 - 40)
        elif (left_x0 == 0 and right_x0 != 0):
            y_offset = np.int32(right_x0 - 1160)
        else:
            y_offset = np.int32(predict_direct[0][0] - forward_direct[0][0])

        infer_result = InferResult()
        infer_result.lanes_y_coords = lane_y_coords
        infer_result.lanes_x_coords = lanes_x_coords
        infer_result.lanes_x_coords_kl = lanes_x_coords_kl
        infer_result.lane_center_x_coords = lane_center_x_coords
        infer_result.forward_direct = forward_direct
        infer_result.predict_direct = predict_direct
        infer_result.y_offset = y_offset
        infer_result.z_offset = z_offset

        return infer_result

    @staticmethod
    def phrasing_output(
        model_out: np.ndarray,
        img_h: int,
        img_w: int,
        griding_num: int,
        cls_num_per_lane: int,
        row_anchor: list,
    ):
        """
        解析 UFLD 网络输出
        - `model_out`: shape `[201, 18, 4]`

        ### Returns: 返回坐标从底部到顶部，根据 y 坐标和 x 坐标分别排序的车道线坐标
        - `lanes_x_coords`: `np.ndarray([4, 18], np.int32)` 四个车道线的 x 坐标
        - `lanes_y_coords`: `np.ndarray([18], np.int32)`    y 坐标
        """
        col_sample = np.linspace(0, 800 - 1, griding_num)
        col_sample_w = col_sample[1] - col_sample[0]

        # flip_updown
        model_out = model_out[:, ::-1, :]

        # relative localization -> absolute position
        prob = softmax(model_out[:-1, :, :], axis=0)
        idx = np.arange(griding_num) + 1
        idx = idx.reshape(-1, 1, 1)
        loc = np.sum(prob * idx, axis=0)
        model_out = np.argmax(model_out, axis=0)
        loc[model_out == griding_num] = 0
        model_out = loc # shape [18,4]

        lanes_x_coords = np.zeros([model_out.shape[1], model_out.shape[0]], dtype=np.int32) # [18, 4]
        lanes_y_coords = np.zeros(model_out.shape[0], dtype=np.int32)                       # [18]

        for i in range(model_out.shape[1]):
            if np.sum(model_out[:, i] != 0) > 2:
                for k in range(model_out.shape[0]):
                    y = int(img_h / 288 * (row_anchor[cls_num_per_lane - 1 - k])) - 1
                    lanes_y_coords[k] = y
                    if model_out[k, i] > 0:

                        x = int(model_out[k, i] * col_sample_w * img_w / 800) - 1

                        # i: lane_id [0,3]
                        # k: row_id  [0,17]
                        lanes_x_coords[i, k] = x

        return (lanes_x_coords, lanes_y_coords)

    @staticmethod
    def fit_and_resample_lanes(
        lx_s: np.ndarray,       # np.int32 [18] left__lane_x_coords
        rx_s: np.ndarray,       # np.int32 [18] right_lane_x_coords
        y_s: np.ndarray,        # np.int32 [18] lane_y_coords
        deg: int = 2,           # 拟合次数
    ):
        """
        返回左右车道线的拟合后重新采样的点: `[resample_lx_s, resample_rx_s ]`
        点不够时，返回 全 0 ，表明该检测结果不可靠
        """

        # 左右车道线 提取非 0 的点的下标
        non_zero_lx_idx = [i for i, lx in enumerate(lx_s) if lx != 0]
        non_zero_rx_idx = [i for i, rx in enumerate(rx_s) if rx != 0]

        def resample(idx, x_s, y_s):
            # 3次多项式拟合，重新采样的车道线
            if len(idx) > 4:
                params = np.polyfit(
                    np.array([y_s[i] for i in idx], dtype=np.int32),
                    np.array([x_s[i] for i in idx], dtype=np.int32),
                    deg,
                )
                resample_x_s = np.polyval(params, y_s).astype(np.int32)
            else:
                resample_x_s = np.zeros_like(x_s, np.int32)
            return resample_x_s

        return [
            resample(non_zero_lx_idx, lx_s, y_s),
            resample(non_zero_rx_idx, rx_s, y_s),
        ]

    @staticmethod
    def compute_center_lane(
        lx_s: np.ndarray,    # np.int32 [18] left__lane_x_coords
        rx_s: np.ndarray,    # np.int32 [18] right_lane_x_coords
        y_s: np.ndarray,     # np.int32 [18] lane_y_coords
    ):
        """ 计算中心车道线 """
        lane_center_x_coords = np.zeros(lx_s.shape, dtype=np.int32)
        for i in range(y_s.shape[0]):
            left__x = lx_s[i]
            right_x = rx_s[i]
            y = y_s[i]

            # 如果左右车道线有一个不存在，则中心车道线也不存在
            if left__x == 0 or right_x == 0:
                lane_center_x_coords[i] = 0
            else:
                lane_center_x_coords[i] = int((left__x + right_x) / 2)
        return lane_center_x_coords

    @staticmethod
    def compute_slope(
        lane_x_coords: np.ndarray, # np.int32 [18] lane_center_x_coords
        lane_y_coords: np.ndarray, # np.int32 [18] lane_y_coords
    ):
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
            # TODO: RankWarning: Polyfit may be poorly conditioned
            a, b, c = np.polyfit(y, x, 2)

            y0 = y[0]
            y1 = y[4]

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

                slope = np.float64((x1 - x0) / (y0 - y1))
            return (slope, (int(x0), y0), (int(x1), y1))

    @staticmethod
    def mark_result(img: np.ndarray, infer_result: InferResult):

        # [18]     所有车道线共用一组 y 坐标
        lane_y_coords = infer_result.lanes_y_coords
        # [4, 18]  4 个车道线的 x 坐标
        lanes_x_coords = infer_result.lanes_x_coords
        # [4, 18]  卡尔曼滤波后 4 个车道线的 x 坐标
        lanes_x_coords_kl = infer_result.lanes_x_coords_kl
        # [18]     中心车道线的 x 坐标
        lane_center_x_coords = infer_result.lane_center_x_coords
        # [2, 2]   实际前进方向
        forward_direct = infer_result.forward_direct
        # [2, 2]   预测前进方向
        predict_direct = infer_result.predict_direct
        y_offset = infer_result.y_offset
        z_offset = infer_result.z_offset

        img_h, img_w, _ = img.shape

        lane_num = lanes_x_coords.shape[0]

        # =============
        # 绘制车道线 点
        # =============
        def mark_lane(
            img: np.ndarray,
            lane_x_coords: np.ndarray,
            lane_y_coords: np.ndarray,
            color: tuple = (0, 0, 255),
            thickness=-1,
        ):
            for i in range(lane_y_coords.shape[0]):
                x = int(lane_x_coords[i])
                y = int(lane_y_coords[i])
                if x != 0:
                    cv2.circle(img, (x, y), 5, color, thickness)
            return img

        for i in range(lane_num):
            img = mark_lane(img, lanes_x_coords[i], lane_y_coords, COLOR_LIST[i], 5)
            img = mark_lane(img, lanes_x_coords_kl[i], lane_y_coords, COLOR_LIST[i + lane_num])

        # =============
        # 连线车道线 连线
        # =============
        def connect_lane(
            img: np.ndarray,
            lane_x_coords: np.ndarray,
            lane_y_coords: np.ndarray,
            color: tuple = (0, 0, 255),
            thickness=2,
        ):
            non_zero_index = np.nonzero(lane_x_coords)  # 找到非零的点，记录下索引
            non_zero_x_coords = lane_x_coords[non_zero_index]
            non_zero_y_coords = lane_y_coords[non_zero_index]
            for j in range(non_zero_x_coords.shape[0] - 1):
                p0 = (non_zero_x_coords[j], non_zero_y_coords[j])
                p1 = (non_zero_x_coords[j + 1], non_zero_y_coords[j + 1])
                cv2.line(img, p0, p1, color, thickness)
            return img

        for i in range(lane_num):
            img = connect_lane(img, lanes_x_coords[i], lane_y_coords, COLOR_LIST[i], 1)
            img = connect_lane(img, lanes_x_coords_kl[i], lane_y_coords, COLOR_LIST[i + lane_num])

        # =============
        # 行驶区域绘制
        # =============
        # non zero 找到非零的点，记录下索引
        ll_n0_idxs = np.nonzero(lanes_x_coords[1]) # ll lane left
        lr_n0_idxs = np.nonzero(lanes_x_coords[2]) # lr lane right

        n0_ll_x_coords = lanes_x_coords[1][ll_n0_idxs]
        n0_ll_y_coords = lane_y_coords[ll_n0_idxs]
        ll_pts = [[int(x), int(y)] for x, y in zip(n0_ll_x_coords, n0_ll_y_coords)]

        n0_lr_x_coords = lanes_x_coords[2][lr_n0_idxs]
        n0_lr_y_coords = lane_y_coords[lr_n0_idxs]
        lr_pts = [[int(x), int(y)] for x, y in zip(n0_lr_x_coords, n0_lr_y_coords)]

        pts = ll_pts + lr_pts[::-1] # 逆序
        if len(pts) > 0:
            pts_np = np.array(pts, dtype=np.int32)
            mask = np.zeros_like(img, dtype=np.uint8)
            cv2.fillPoly(mask, [pts_np], (0, 139, 0))
            img = cv2.addWeighted(img, 1, mask, 0.2, 0)

        # =============
        # 绘制中心车道线
        # =============
        img = mark_lane(img, lane_center_x_coords, lane_y_coords, COLOR_LIST[lane_num + lane_num + 1])
        img = cv2.line(img, tuple(forward_direct[0]), tuple(forward_direct[1]), COLOR_LIST[lane_num * 2 + 2], 2)

        # 绘制预测方向(延长)
        if predict_direct[0][1] != 0 and predict_direct[1][1] != 0:
            a, b = np.polyfit(predict_direct[:, 1], predict_direct[:, 0], 1)
            y0 = forward_direct[0][1]
            y1 = forward_direct[1][1]
            predict_direct = np.array(((int(a * y0 + b), y0), (int(a * y1 + b), y1)), dtype=np.int32)
        img = cv2.line(img, tuple(predict_direct[0]), tuple(predict_direct[1]), COLOR_LIST[lane_num * 2 + 3], 2)

        # 在图像中输出图像尺寸，偏移距离，预测方向的斜率
        color = (255, 255, 255)
        for line, text in enumerate(
            [
                f"size  : {img_w},{img_h}", f"y_offset : {y_offset}",
                f"z_offset : {z_offset:.2f} ({z_offset*180/3.14159:.2f})",
                f"({lane_y_coords[0]}, {lanes_x_coords[1][0]}, {lane_center_x_coords[0]}, {lanes_x_coords[2][0]})"
            ]
        ):
            # 按行显示，需要根据图像高度调整
            show_line_num = 20  # 总共可以显示的行数，可以修改
            base_scale = 30     # 这个数值越大，字体越小

            line_height = img_h / show_line_num
            scale = line_height / base_scale
            thickness = int(scale * 2)

            img = cv2.putText(
                img,
                text,
                org=(int(0.01 * img_h), int((line + 1) * line_height)),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=scale,
                color=color,
                thickness=thickness,
            )
        return img


class NormalizeValue:
    mean = [0.485, 0.456, 0.406]
    std = [0.229, 0.224, 0.225]


def img2tensor(img: np.ndarray, size: Tuple[int, int] = (800, 288)):
    """
    Transform the input image to the format that the model needs.

    ## Args:
    - `img`: `np.ndarray`, shape: `(H, W, C)`, dtype: `np.uint8`, range: `[0, 255]`

    ## Returns:
    - `img`: `np.ndarray`, shape: `(1, C, H, W)`, dtype: `np.float32`, range: `[0, 1]`
    """

    img = img / 255.0
    img = (img - NormalizeValue.mean) / NormalizeValue.std
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    img = img.astype(np.float32)
    return img


def softmax(x, axis=None):
    x_max = np.amax(x, axis=axis, keepdims=True)
    exp_x_shifted = np.exp(x - x_max)
    return exp_x_shifted / np.sum(exp_x_shifted, axis=axis, keepdims=True)
