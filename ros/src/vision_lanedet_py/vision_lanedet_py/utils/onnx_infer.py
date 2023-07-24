from typing import List
import cv2
import numpy as np
import onnxruntime as ort

from .base_model_infer import BaseModelInfer,InferResult

from .logger import ColorStr
from .constants import (
    GRIDING_NUM,
    CLS_NUM_PER_LANE,
    ROW_ANCHOR,
)


class ONNXInfer(BaseModelInfer):
    def __init__(
        self,
        onnx_file,
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
        super().__init__(griding_num, cls_num_per_lane, row_anchor, klf_id)

        self.ort_session = ort.InferenceSession(
            onnx_file, providers=[
                'TensorrtExecutionProvider',
                'CUDAExecutionProvider',
                'CPUExecutionProvider',
            ]
        )
        binding__input_names = [binding.name for binding in self.ort_session.get_inputs()]
        binding__input_shapes = [binding.shape for binding in self.ort_session.get_inputs()]
        binding__input_types = [binding.type for binding in self.ort_session.get_inputs()]

        binding_output_names = [binding.name for binding in self.ort_session.get_outputs()]
        binding_output_shapes = [binding.shape for binding in self.ort_session.get_outputs()]
        binding_output_types = [binding.type for binding in self.ort_session.get_outputs()]\

        print(ColorStr.info("Parsing ONNX info:"))
        print(ColorStr.info("  - providers:"), self.ort_session.get_providers())
        print(ColorStr.info("  --- inputs:"), binding__input_names)
        print(ColorStr.info("       -- names:"), binding__input_names)
        print(ColorStr.info("       - shapes:"), binding__input_shapes)
        print(ColorStr.info("       -- types:"), binding__input_types)
        print(ColorStr.info("  --- outputs:"), binding_output_names)
        print(ColorStr.info("       -- names:"), binding_output_shapes)
        print(ColorStr.info("       - shapes:"), binding_output_shapes)
        print(ColorStr.info("       -- types:"), binding_output_types)

        if "input" not in [binding.name for binding in self.ort_session.get_inputs()]:
            raise ValueError("'input'  not found in ONNX model, expected one of", binding__input_names)
        if "output" not in [binding.name for binding in self.ort_session.get_outputs()]:
            raise ValueError("'output' not found in ONNX model, expected one of", binding_output_names)

    def infer_model(self, input: np.ndarray)->np.ndarray:
        """
        # 执行模型推理，对 ONNX 模型进行推理的封装
        ## Args:
        - `input`: `np.ndarray`, `(1, C, H, W)`, `np.float32`, range: `[0, 1]`
        
        ## Returns:
        - `output`: `np.ndarray`, `(201, 18, 4)`, `np.float32`, range: `[0, 1]`
        """
        outputs: List[np.ndarray] = self.ort_session.run(
            ["output"],
            {"input": input},
        )
        output = outputs[0]
        output = output.squeeze()
        return output
