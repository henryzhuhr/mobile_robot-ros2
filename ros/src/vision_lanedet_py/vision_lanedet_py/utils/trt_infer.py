import numpy as np
assert np.__version__ == "1.20.2", "numpy version must be 1.20.2"

from .base_model_infer import BaseModelInfer, InferResult

from .logger import ColorStr
from .constants import (
    GRIDING_NUM,
    CLS_NUM_PER_LANE,
    ROW_ANCHOR,
)

import tensorrt as trt # Test on TRT 8.2.1
assert trt.__version__ == "8.2.1.8", "tensorrt version must be 8.2.1"
import pycuda.driver as cuda
import pycuda.autoinit # This is needed for initializing CUDA driver


class TensorRTInfer(BaseModelInfer):
    """
    # Test on TensorRT 8.2.1
    """
    def __init__(
        self,
        weight_file,
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

        # Load TRT engine
        self.logger = trt.Logger(trt.Logger.ERROR)
        trt.init_libnvinfer_plugins(self.logger, namespace="")

        with open(weight_file, "rb") as f, trt.Runtime(self.logger) as runtime:
            assert runtime
            self.engine = runtime.deserialize_cuda_engine(f.read()) # 反序列化模型
        assert self.engine

        self.context = self.engine.create_execution_context() # context推理进程(相当于CPU中的一个进程)
                                                              # self.context.all_binding_shapes_specified == True 时候，确认所有绑定的输入输出张量形状均被指定 。
        assert self.context

        # self.engine.num_bindings, 引擎中绑定的张量数量，是导出 ONNX 时指定的 name: ["input"]/["output"]

        # Setup I/O bindings: 绑定了两个内容，输入/输出 ["input"]/["output"]
        # names = []
        # for i in range(self.engine.num_bindings):
        #     names.append(self.engine.get_tensor_name(i))

        # 解析引擎中的输入输出张量
        # {'index': 1, 'name': 'input', 'dtype': dtype('float32'), '
        # shape': [1, 3, 288, 800], 'allocation': 140469354889216, 'size': 2764800}
        self.inputs = []
        # {'index': 1, 'name': 'output', 'dtype': dtype('float32'),
        # 'shape': [1, 201, 18, 4], 'allocation': 140469395429376, 'size': 57888}
        self.outputs = []
        self.allocations = []
        for i in range(self.engine.num_bindings):
            is_input = False
            if self.engine.binding_is_input(i):
                is_input = True
            name = self.engine.get_binding_name(i)
            dtype = self.engine.get_binding_dtype(i)
            shape = self.engine.get_binding_shape(i)
            if is_input:
                self.batch_size = shape[0]
            size = np.dtype(trt.nptype(dtype)).itemsize
            for s in shape:
                size *= s
            allocation = cuda.mem_alloc(size)
            binding = {
                'index': i,
                'name': name,
                'dtype': np.dtype(trt.nptype(dtype)),
                'shape': list(shape),
                'allocation': allocation,
            }
            self.allocations.append(allocation)
            if self.engine.binding_is_input(i):
                self.inputs.append(binding)
            else:
                self.outputs.append(binding)
        assert self.batch_size > 0
        assert len(self.inputs) > 0
        assert len(self.outputs) > 0
        assert len(self.allocations) > 0

    def infer_model(self, input: np.ndarray) -> np.ndarray:
        """
        # 执行模型推理，对 TensorRT 模型进行推理的封装
        ## Args:
        - `input`: `np.ndarray`, `(1, C, H, W)`, `np.float32`, range: `[0, 1]`
        
        ## Returns:
        - `output`: `np.ndarray`, `(201, 18, 4)`, `np.float32`, range: `[0, 1]`
        """
        """
        Execute inference on a batch of images. The images should already be batched and preprocessed, as prepared by
        the ImageBatcher class. Memory copying to and from the GPU device will be performed here.
        :param batch: A numpy array holding the image batch.
        :param scales: The image resize scales for each image in this batch. Default: No scale postprocessing applied.
        :return: A nested list for each image in the batch and each detection in the list.
        """

        batch = input

        # Prepare the output data.
        outputs = []
        for shape, dtype in self.output_spec():
            outputs.append(np.zeros(shape, dtype))
        # Process I/O and execute the network.
        cuda.memcpy_htod(self.inputs[0]['allocation'], np.ascontiguousarray(batch))

        self.context.execute_v2(self.allocations)
        for o in range(len(outputs)):
            cuda.memcpy_dtoh(outputs[o], self.outputs[o]['allocation'])

        # Process the results.
        out = outputs[0]
        out = np.squeeze(out)
        return out

    def input_spec(self):
        """
        Get the specs for the input tensor of the network. Useful to prepare memory allocations.
        :return: Two items, the shape of the input tensor and its (numpy) datatype.
        """
        return self.inputs[0]['shape'], self.inputs[0]['dtype']

    def output_spec(self):
        """
        Get the specs for the output tensors of the network. Useful to prepare memory allocations.
        :return: A list with two items per element, the shape and (numpy) datatype of each output tensor.
        """
        specs = []
        for o in self.outputs:
            specs.append((o['shape'], o['dtype']))
        return specs