import os
import shutil
from typing import List
import cv2
import numpy as np

SAVE_IMG_NUM = 5  # 保存图像数目


class BOARD_SIZE:  # 标定板交叉点的个数
    W = 9  # 列
    H = 6  # 行


CheckerboardSize = 10  # 单位mm

SAVE_DIR = "tmp/distortion-correction"


def main():
    img_list: List[cv2.Mat] = []

    if not os.path.exists(SAVE_DIR):
        video_capture = cv2.VideoCapture(2)
        print("Press 's' to save image, 'q' to quit")
        while True:
            ret, frame = video_capture.read()
            if not ret:
                print("No frame")
                break

            cv2.imshow("frame", frame)

            if cv2.waitKey(1) & 0xFF == ord("s"):
                img_list.append(frame)
                print("Image saved", len(img_list))
                if len(img_list) >= SAVE_IMG_NUM:
                    break
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        os.makedirs(SAVE_DIR, exist_ok=True)
        for i, img in enumerate(img_list):
            cv2.imwrite(f"{SAVE_DIR}/img-{i}.png", img)
        video_capture.release()
    else:
        for file in os.listdir(SAVE_DIR):
            if file.endswith(".png"):
                img = cv2.imread(f"{SAVE_DIR}/{file}")
                img_list.append(img)

    OUT_DIR = "tmp/distortion-correction-out"
    os.makedirs(OUT_DIR, exist_ok=True)

    # 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)，去掉Z坐标，记为二维矩阵，认为在棋盘格这个平面上Z=0
    # 构造0矩阵，用于存放角点的世界坐标
    objp = np.zeros((BOARD_SIZE.W * BOARD_SIZE.H, 3), np.float32)
    objp[:, :2] = np.mgrid[0: BOARD_SIZE.W, 0: BOARD_SIZE.H].T.reshape(
        -1, 2
    )  # 三维网格坐标划分

    dc_list: List[cv2.Mat] = []  # distortion correction 去畸变后的图像

    obj_points: List[cv2.Mat] = []  # 3d points in real world space
    img_points: List[cv2.Mat] = []  # 2d points in image plane.

    for i in range(len(img_list)):
        img = img_list[i].copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners 粗略找到棋盘格角点
        # 这里找到的是这张图片中角点的亚像素点位置
        ret, corners = cv2.findChessboardCorners(
            gray, (BOARD_SIZE.W, BOARD_SIZE.H), None)

        # If found, add object points, image points
        if ret:
            # 精确找到角点坐标 亚像素级角点检测
            corners = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                criteria=(  # 循环中断条件
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    10,  # CheckerboardSize 标定的棋盘格尺寸,必须为整数.(单位:mm或0.1mm)
                    0.001,
                ),
            )
            obj_points.append(objp)
            img_points.append(corners)
            cv2.drawChessboardCorners(
                img, (BOARD_SIZE.W, BOARD_SIZE.H), corners, ret)
            # cv2.imwrite(f"{OUT_DIR}/img-{i}.png", img)
            dc_list.append([img_list[i], img])
            cv2.waitKey(0)

    (ret, mtx,  # 内参数矩阵
     dist,  # dist畸变值 # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)
     rvecs,  # 旋转向量  # 外参数
     tvecs,  # 平移向量  # 外参数
     ) = cv2.calibrateCamera(
        obj_points, img_points, gray.shape[::-1], None, None)

    print("dist", dist)
    print("mtx", mtx)
    print("rvecs", rvecs)
    print("tvecs", tvecs)

    for i, (img, img_marked) in enumerate(dc_list):
        h, w = img.shape[:2]
        # 矫正图像
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            mtx, dist, (w, h), 0, (w, h)
        )
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        # 根据前面ROI区域裁剪图片
        # x, y, w, h = roi
        # dst = dst[y: y + h, x: x + w]

        save_img = cv2.hconcat([img_marked, dst])

        cv2.imwrite(f"{OUT_DIR}/undist-{i}.jpg", save_img)


def undistort(frame):
    """ https://juejin.cn/post/7094064438243753991 """
    """ 
    :return mtx: 内参数矩阵.{f_x}{0}{c_x}{0} {f_y}{c_y} {0}{0}{1}
    :return dist: 畸变系数.(k_1,k_2,p_1,p_2,k_3)

    dist [[ 6.42963420e+00 -1.89712503e+03 -1.88436634e-01  2.39765548e-01 1.01743223e+01]]
    mtx [[3.83629477e+03 0.00000000e+00 2.37889070e+02]
    [0.00000000e+00 6.03926756e+03 2.56106908e+02]
    [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
    rvecs (array([[-0.81799596],
        [ 0.013956  ],
        [ 0.05007646]]), array([[-0.80196288],
        [ 0.02611448],
        [ 0.06106248]]), array([[-0.82469471],
        [-0.03721813],
        [-0.07417017]]), array([[-0.85456684],
        [-0.00342071],
        [ 0.00631013]]), array([[-0.87322278],
        [ 0.01668703],
        [ 0.00773101]]))
    tvecs (array([[  3.21060465],
        [ -2.56507886],
        [162.69868378]]), array([[  5.67996147],
        [ -2.33709269],
        [166.46837721]]), array([[ -7.5895274 ],
        [ -2.03495274],
        [159.83672634]]), array([[ -0.48274804],
        [ -2.36807616],
        [169.45850389]]), array([[ -1.31472648],
        [ -0.25045361],
       """
    fx = 685.646752
    cx = 649.107905
    fy = 676.658033
    cy = 338.054431
    k1, k2, p1, p2, k3 = -0.363219, 0.093818, 0.006178, -0.003714, 0.0

    # 相机坐标系到像素坐标系的转换矩阵
    k = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ])
    # 畸变系数
    d = np.array([
        k1, k2, p1, p2, k3
    ])
    h, w = frame.shape[:2]
    mapx, mapy = cv2.initUndistortRectifyMap(k, d, None, k, (w, h), 5)
    return cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)


if __name__ == "__main__":
    main()
