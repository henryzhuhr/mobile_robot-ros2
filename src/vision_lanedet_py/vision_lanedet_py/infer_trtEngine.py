import os

cuda_home = os.getenv("CUDA_HOME")
tensorrt_home = os.getenv("TENSORRT_HOME")
assert os.path.exists(cuda_home), f"environment variable \"CUDA_HOME\" not exists"
assert os.path.exists(tensorrt_home), f"environment variable \"TENSORRT_HOME\" not exists"
os.environ["PATH"] = ":".join([os.getenv("PATH"), f"{tensorrt_home}/bin", f"{cuda_home}/bin"])
os.environ["LD_LIBRARY_PATH"] = ":".join([os.getenv("LD_LIBRARY_PATH"), f"{tensorrt_home}/lib", f"{cuda_home}/lib64"])

import time
import cv2
from .utils.trt_infer import TensorRTInfer, InferResult


class Args:
    """需要配置的参数"""
    weight_file = f"{os.getcwd()}/weights/ufld-final-x64.engine"
    video = f"{os.getcwd()}/test.mp4"


def main():
    model_infer = TensorRTInfer(Args.weight_file)

    cap = cv2.VideoCapture(Args.video)
    skip_frame = 5 # 每隔 skip_frame 帧进行一次推理
    i = 0
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            i += 1

            # 缩放图像，使得图像的高度为 720，宽度按比例缩放。高度可以根据实际需要调整
            img_h = 720
            img_w = frame.shape[1] * img_h // frame.shape[0]
            img = cv2.resize(frame, (img_w, img_h))
            if i % skip_frame == 0:
                st = time.time()

                infer_result: InferResult = model_infer.infer(img)

                lane_y_coords = infer_result.lanes_y_coords              # [18]     所有车道线共用一组 y 坐标
                lanes_x_coords = infer_result.lanes_x_coords             # [4, 18]  4 个车道线的 x 坐标
                lanes_x_coords_kl = infer_result.lanes_x_coords_kl       # [4, 18]  卡尔曼滤波后 4 个车道线的 x 坐标
                lane_center_x_coords = infer_result.lane_center_x_coords # [18]     中心车道线的 x 坐标
                forward_direct = infer_result.forward_direct             # [2, 2]   实际前进方向
                predict_direct = infer_result.predict_direct             # [2, 2]   预测前进方向
                slope = infer_result.slope                               # 预测方向的斜率
                offset_distance = infer_result.offset_distance           # 偏移距离

                # 推理时间
                infer_time = (time.time() - st) * 1000
                print("time: %.4f ms" % infer_time)

                if True: # 如果不需要绘制，可以改成 False
                    model_infer.mark_result(img, infer_result)

            cv2.imshow("img", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()