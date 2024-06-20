"""
运行代码，使用鼠标左击frame弹出框获取HSV值
说明：
    HSV值: [H->色调, S->饱和度, V->亮度]
"""

import cv2


def get_hsv_value(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_value = hsv[y, x]
        print(f"HSV值: {list(hsv_value)}")


# 读取图像
camera_id = 0

cap = cv2.VideoCapture(camera_id)
if not cap.isOpened():
    raise ValueError(f"Cannot open camera {camera_id}")
ret, frame = cap.read()
if not ret:
    raise ValueError(f"Can't receive frame from camera {camera_id}")

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

cv2.imshow("frame", frame)
cv2.setMouseCallback("frame", get_hsv_value)

cv2.waitKey(0)
cv2.destroyAllWindows()

# 25, 83, 100
