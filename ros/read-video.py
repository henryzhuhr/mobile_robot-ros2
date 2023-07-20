import cv2

cap=cv2.VideoCapture(0)
# 查看usb摄像头的帧率
fps=cap.get(cv2.CAP_PROP_FPS)
print(fps)
# 设置摄像头的帧率
cap.set(cv2.CAP_PROP_FPS,30)

fps=cap.get(cv2.CAP_PROP_FPS)
print(fps)

# width=cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
# height=cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)

while True:
    ret,frame=cap.read()
    if not ret:
        break
    # print(frame.shape,[width,height])

    cv2.imshow("img",frame)

    cv2.waitKey(1)

cap.release()