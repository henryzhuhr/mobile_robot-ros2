class ButtonState:
    """按键状态记录"""

    def __init__(self) -> None:
        self.last_state = 0  # 记录上一次状态
        self.flip_cnt = 0  # 状态翻转计数
