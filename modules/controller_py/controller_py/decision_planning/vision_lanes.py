import numpy as np


class LanesDetectionDecision:
    def __init__(self) -> None:
        self.lanes_y = np.zeros([18], dtype=np.int32)
        self.lanes_x = np.zeros([4, 18], dtype=np.int32)

    def speed_correction(self,
                         height: int, width: int,
                         lane_y_coords: np.ndarray,  # [18]    np.int32
                         lanes_x_coords: np.ndarray,  # [4, 18] np.int32
                         ):
        (
            left__lane_x,                # 重新拟合左车道线
            right_lane_x,                # 重新拟合右车道线
        ) = self.fit_and_resample_lanes(
            lanes_x_coords[1],
            lanes_x_coords[2],
            lane_y_coords,
        )
        # 计算中心线的 x 坐标
        lane_center_x_coords = self.compute_center_lane(
            lanes_x_coords[1],
            lanes_x_coords[2],
            lane_y_coords,
        )
        # 计算预测方向的斜率
        (
            slope,    # 斜率
            (x0, y0),  # 中心车道线，最底部的点
            (x1, y1),  # 中心车道线，最底部的上一个点
        ) = self.compute_slope(lane_center_x_coords, lane_y_coords)

        # 计算方向角便宜
        z_offset = np.arctan(slope)

        # 实际的前进方向
        # TODO ，需要实际标定，目前使用图像中心线作为前进方向
        forward_direct = np.array(
            ((width // 2, height), (width // 2, height / 5)), dtype=np.int32)
        # 预测的前进方向
        predict_direct = np.array(((x0, y0), (x1, y1)), dtype=np.int32)

        # TODO: 预测方向于左右的距离，需要实际标定

        # 计算偏移距离，取最底部的点的 x 坐标计算，用像素表示，(以图像中心为原点，向右为正方向)，
        # 对应到小车就是 y 轴方向
        if x0 == 0:
            y_offset = np.int32(0)
        else:
            y_offset = np.int32(predict_direct[0][0] - forward_direct[0][0])

        return (y_offset, z_offset)

    @staticmethod
    def fit_and_resample_lanes(
        lx_s: np.ndarray,       # np.int32 [18] left__lane_x_coords
        rx_s: np.ndarray,       # np.int32 [18] right_lane_x_coords
        y_s: np.ndarray,        # np.int32 [18] lane_y_coords
        deg: int = 2,           # 拟合次数
    ):
        """
        返回左右车道线的拟合后重新采样的点: `[resample_lx_s, resample_rx_s ]`
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
            x = np.array([lane_x_coords[i]
                         for i in valid_point_index], dtype=np.int32)
            y = np.array([lane_y_coords[i]
                         for i in valid_point_index], dtype=np.int32)

            # 根据有效点拟合二次曲线
            # TODO: RankWarning: Polyfit may be poorly conditioned
            a, b, c = np.polyfit(y, x, 2)

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

                slope = np.float64((x1 - x0) / (y0 - y1))
            return (slope, (int(x0), y0), (int(x1), y1))
