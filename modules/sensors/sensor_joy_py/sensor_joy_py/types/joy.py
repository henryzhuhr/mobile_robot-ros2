from typing import Dict, List
import json

DEFAULT_MAP = {
    "axis_stick_left__LR": 0,
    "axis_stick_left__UD": 1,
    "axis_stick_right_LR": 3,
    "axis_stick_right_UD": 4,
    "axis_cross_LR": 6,
    "axis_cross_UD": 7,
    "axis_LT": 2,
    "axis_RT": 5,
    "button_A": 0,
    "button_B": 1,
    "button_X": 2,
    "button_Y": 3,
    "button_LB": 4,
    "button_RB": 5,
    "button_select": 6,
    "button_start": 7,
    "button_stick__left": 8,
    "button_stick_right": 9
}


class JOY_KB_MAP:

    def __init__(self, config_file: str = None) -> None:
        try:
            with open(config_file, "r") as f:
                config_dict = json.load(f)
            for key in DEFAULT_MAP:
                assert key in config_dict
                assert isinstance(config_dict[key], int)
        except:
            config_dict = DEFAULT_MAP

        self.axis_stick_left__LR = config_dict["axis_stick_left__LR"]
        self.axis_stick_left__UD = config_dict["axis_stick_left__UD"]
        self.axis_stick_right_LR = config_dict["axis_stick_right_LR"]
        self.axis_stick_right_UD = config_dict["axis_stick_right_UD"]
        self.axis_cross_LR = config_dict["axis_cross_LR"]
        self.axis_cross_UD = config_dict["axis_cross_UD"]
        self.axis_LT = config_dict["axis_LT"]
        self.axis_RT = config_dict["axis_RT"]
        self.button_A = config_dict["button_A"]
        self.button_B = config_dict["button_B"]
        self.button_X = config_dict["button_X"]
        self.button_Y = config_dict["button_Y"]
        self.button_LB = config_dict["button_LB"]
        self.button_RB = config_dict["button_RB"]
        self.button_select = config_dict["button_select"]
        self.button_start = config_dict["button_start"]
        self.button_stick__left = config_dict["button_stick__left"]
        self.button_stick_right = config_dict["button_stick_right"]
        self.config_dict = config_dict

    def __str__(self) -> str:
        return " ".join([f"{key}:{self.config_dict[key]}" for key in DEFAULT_MAP.keys()])
