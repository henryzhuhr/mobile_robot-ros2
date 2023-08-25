class SpeedState:
    """
    速度状态

    - `x` : x 轴方向速度 (m/s)
    - `y` : y 轴方向速度 (m/s)
    - `z` : z 轴方向速度 (rad/s) 航向角
    """

    def __init__(
        self,
            x: float = 0.0,
            y: float = 0.0,
            z: float = 0.0,
    ) -> None:
        self.x = x
        self.y = y
        self.z = z
