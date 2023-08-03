from typing import Tuple
import cv2
import numpy as np


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
                if model_out[k, i] > 0:

                    x = int(model_out[k, i] * col_sample_w * img_w / 800) - 1
                    y = int(img_h / 288 * (row_anchor[cls_num_per_lane - 1 - k])) - 1
                    # i: lane_id [0,3]
                    # k: row_id  [0,17]
                    lanes_x_coords[i, k] = x
                    lanes_y_coords[k] = y

    return (lanes_x_coords, lanes_y_coords)
