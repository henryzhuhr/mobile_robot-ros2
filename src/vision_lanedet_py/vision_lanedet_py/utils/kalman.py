import numpy as np
import cv2


class LaneKalmanFiter:
    def __init__(self):
        # 初始化卡尔曼滤波器
        """
        如果你需要对整个线进行滤波，而不仅仅是点，你可以考虑将线模型化为一系列参数，然后使用卡尔曼滤波来跟踪和预测这些参数。例如，如果你将线建模为一条二次曲线 y = ax^2 + bx + c，你就可以跟踪参数 a、b 和 c 的变化。
        状态变量：这些是系统在时间内发展的变量，也就是我们需要预测或估计的变量。在你的例子中，我们设定了6个状态变量。这些状态变量包括了二次函数的三个系数（a, b, c），以及它们的一阶差分（即速度）。所以我们有 a，b，c，da/dt，db/dt，dc/dt 共6个状态变量。

        测量变量：这些是我们可以直接从系统中测量到的变量。在你的例子中，我们设定了3个测量变量，这些测量变量就是我们通过拟合得到的二次函数的系数 a，b 和 c。
        """
        self.kalman = cv2.KalmanFilter(
            6,                                    # 转移矩阵维度为 6
            3,                                    # 测量矩阵维度为 3
        )
        '''
        Kalman这个类需要初始化下面变量：
        转移矩阵，测量矩阵，控制向量(没有的话，就是0)，
        过程噪声协方差矩阵，测量噪声协方差矩阵，
        后验错误协方差矩阵，前一状态校正后的值，当前观察值。
            在此cv2.KalmanFilter(4,2)表示转移矩阵维度为4，测量矩阵维度为2
        卡尔曼滤波模型假设k时刻的真实状态是从(k − 1)时刻的状态演化而来，符合下式：
                    X(k) = F(k) * X(k-1) + B(k)*U(k) + W（k）
        其中
        F(k)  是作用在xk−1上的状态变换模型（/矩阵/矢量）。 
        B(k)  是作用在控制器向量uk上的输入－控制模型。 
        W(k)  是过程噪声，并假定其符合均值为零，协方差矩阵为Qk的多元正态分布。
        '''
                                                  # 测量矩阵
        self.kalman.measurementMatrix = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
            ], dtype=np.float32
        )

        # 转移矩阵
        self.kalman.transitionMatrix = np.array(
            [
                [1, 0, 0, 1, 0, 0],
                [0, 1, 0, 0, 1, 0],
                [0, 0, 1, 0, 0, 1],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ],
            dtype=np.float32
        )

        self.kalman.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-4 # 过程噪声协方差矩阵

        # ax^2 + bx + c 的系数
        self.params = np.zeros([3], dtype=np.float32)

    def update(self, lanes_x_coords: np.ndarray, lanes_y_coords: np.ndarray):
        assert lanes_x_coords.shape == lanes_y_coords.shape
        assert lanes_x_coords.shape[0] == lanes_y_coords.shape[0]
        num_point = lanes_y_coords.shape[0]

        filter_i = [] # remove [0,0], 保存有效的点的索引
        for i in range(lanes_x_coords.shape[0]):
            if lanes_x_coords[i] != 0:
                filter_i.append(i)

        valid_points_num = len(filter_i)
        if (valid_points_num == 0):
            return lanes_x_coords
        else:
            x = np.array([lanes_x_coords[i] for i in filter_i], dtype=np.int32)
            y = np.array([lanes_y_coords[i] for i in filter_i], dtype=np.int32)

            # params = [a,b,c] 使用2次多项式拟合车道线 ax^2 + bx + c
            params = np.polyfit(y, x, 2)

            # 使用卡尔曼滤波器预测状态
            predicted = self.kalman.predict()

            # 测量更新
            measured = np.array(params, dtype=np.float32).reshape(-1, 1)
            self.kalman.correct(measured)

            # 卡尔曼滤波器更新后得到的参数
            self.params = self.kalman.statePost.flatten() # [6,3]Ï
            a = self.params[0]
            b = self.params[1]
            c = self.params[2]

            # 使用参数生成一组 y 值对应的 x 值
            kl_lanes_x_coords = np.zeros(num_point, dtype=np.int32)
            for i in range(num_point):
                y = lanes_y_coords[i]
                x = lanes_x_coords[i]
                x_kl = a * y * y + b * y + c if x != 0 else 0
                kl_lanes_x_coords[i] = int(x_kl)

            return kl_lanes_x_coords

    def update_old(self, lane_points: np.ndarray):
        """
        - `lane`: shape `[18, 2]`
        """
        # 提取x和y坐标
        filter_points_list = [] # remove [0,0]
        for (x, y) in lane_points:
            if x != 0 and y != 0:
                filter_points_list.append([x, y])

        valid_points_num = len(filter_points_list)
        if (valid_points_num == 0):
            return (np.zeros([3], dtype=np.float32), -1, -1, 0)
        else:
            filter_points = np.array(filter_points_list, dtype=np.float32)
            x = filter_points[:, 0]
            y = filter_points[:, 1]

            # 使用三次多项式拟合车道线
            params = np.polyfit(y, x, 2)

            # 使用卡尔曼滤波器预测状态
            predicted = self.kalman.predict()
            # print(f"Predicted parameters: {predicted}")

            # 测量更新
            measured = np.array([[params[0]], [params[1]], [params[2]]], dtype=np.float32)
            self.kalman.correct(measured)

            # 卡尔曼滤波器更新后得到的参数
            self.params = self.kalman.statePost.flatten()

            # 使用参数生成一组 y 值对应的 x 值
            # y_values = np.linspace(min(y), max(y), valid_points_num) # 可以根据你的实际需要调整
            # x_values = params[0] * y_values**2 + params[1] * y_values + params[2]
            # filtered_lane_points = np.array([x_values, y_values]).T
            return (
                self.params,
                min(y),
                max(y),
                valid_points_num,
            )