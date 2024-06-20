"""
ArUco 生成: https://chev.me/arucogen/
"""
import json
from typing import Tuple
import socket
import cv2
import cv2.aruco as aruco
import numpy as np
import time


# Camera 类
class Camera:
    def __init__(self, camera_id: int):
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise ValueError(f"Cannot open camera {camera_id}")

    def read_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise ValueError(f"Can't receive frame from camera {self.camera_id}")
        return frame

    def release(self):
        self.cap.release()


# ArUco 检测器
class ArUcoDetector:
    def __init__(self, marker_id: int):
        self.marker_id = marker_id
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 5)
        corners, ids, rejected = self.detector.detectMarkers(blur)
        return corners, ids

    def draw_markers(self, frame, corners, ids):
        aruco.drawDetectedMarkers(frame, corners, ids)


# 圆形检测器
class CircleDetector:
    def __init__(self, hsv_color, param1, param2, min_dist, min_radius, max_radius):

        self.lower_color = np.array([
            max(0, hsv_color[0] - 10),
            max(0, hsv_color[1] - 40),
            max(0, hsv_color[2] - 40)
        ])
        self.upper_color = np.array([
            min(255, hsv_color[0] + 10),
            min(255, hsv_color[1] + 40),
            min(255, hsv_color[2] + 40)
        ])

        self.param1 = param1
        self.param2 = param2
        self.min_dist = min_dist
        self.min_radius = min_radius
        self.max_radius = max_radius

    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        color_only = cv2.bitwise_and(frame, frame, mask=mask)
        gray = cv2.cvtColor(color_only, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 5)

        hough_circles = cv2.HoughCircles(
            blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=self.min_dist,
            param1=self.param1, param2=self.param2, minRadius=self.min_radius, maxRadius=self.max_radius
        )
        return hough_circles

    @staticmethod
    def draw_circles(frame, hough_circles):
        if hough_circles is not None:
            circles = np.round(hough_circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                cv2.putText(frame, f"R={r}", (x - r, y - r - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


# socket 通信类
class UdpClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, message: str):
        """发送数据"""
        self.sock.sendto(message.encode('utf-8'), (self.host, self.port))

    def close(self):
        """关闭连接"""
        self.sock.close()

    def __del__(self):
        self.close()


# 控制类
class Controller:
    def __init__(self, camera_id, aruco_id,
                 hsv_color, param1, param2, min_dist, min_radius, max_radius,
                 esp32_ip=None, esp32_port=None,
                 flask_ip=None, flask_port=None):
        self.camera = Camera(camera_id)
        self.aruco_detector = ArUcoDetector(aruco_id)
        self.circle_detector = CircleDetector(hsv_color, param1, param2, min_dist, min_radius, max_radius)
        if esp32_ip and esp32_port:
            self.esp32_communication = UdpClient(esp32_ip, esp32_port)
        else:
            self.esp32_communication = None
        if flask_ip and flask_port:
            self.flask_communication = UdpClient(flask_ip, flask_port)
        else:
            self.flask_communication = None

    def calculate_corner_center(self, corner) -> Tuple[int, int]:
        """返回 arUco 的中点"""
        center_x = np.mean(corner[:, 0])
        center_y = np.mean(corner[:, 1])
        return int(center_x), int(center_y)

    def calculate_2points_center(self, point1, point2) -> Tuple[int, int]:
        """返回两个点的中点"""
        center_x = (point1[0] + point2[0]) / 2
        center_y = (point1[1] + point2[1]) / 2
        return int(center_x), int(center_y)

    def draw_lines_by_dots(self, frame, *points):
        """画出点之间的连线"""
        number_of_points = len(points)
        if number_of_points >= 2:
            cv2.circle(frame, points[0], 6, (0, 0, 255), -1)
            for i in range(number_of_points - 1):
                cv2.line(frame, points[i], points[i + 1], (0, 200, 200), 3)
                cv2.circle(frame, points[i + 1], 6, (0, 0, 255), -1)

    def show_angle_and_distance(self, frame, angle, distance):
        """显示角度和距离"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (0, 230, 0)
        thickness = 2
        cv2.putText(frame, f"Angle: {angle:.2f}", (500, 50), font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"Distance: {distance:.2f}", (500, 100), font, font_scale, color, thickness, cv2.LINE_AA)

    def send_data(self, angle, distance):
        command = json.dumps({'angle': round(angle, 2), 'distance': round(distance, 2)})
        if self.esp32_communication:
            self.esp32_communication.send(command)
        if self.flask_communication:
            self.flask_communication.send(command)

    def process_frame(self, frame):
        """发送角度和距离数据到 esp32"""
        corners, ids = self.aruco_detector.detect(frame)
        hough_circles = self.circle_detector.detect(frame)
        self.circle_detector.draw_circles(frame, hough_circles)
        ids_list = []

        if ids is not None:
            ids_list = list(ids.ravel())
            self.aruco_detector.draw_markers(frame, corners, ids)

        if ids is None:
            self.send_data(0, 0)
            time.sleep(0.5)
            return frame

        if (ids is not None) and (self.aruco_detector.marker_id in ids) and (hough_circles is not None):
            corner_car_index = ids_list.index(self.aruco_detector.marker_id)
            circles = np.uint16(np.around(hough_circles))
            corner_car = corners[corner_car_index][0]
            center_car = self.calculate_corner_center(corner_car)
            x, y, r = circles[0, 0]
            center_circle = (x, y)
            center_car_line1 = self.calculate_corner_center(corner_car[0:2])

            vector1 = np.array(center_car) - np.array(center_car_line1)
            vector2 = np.array(center_car_line1) - np.array(center_circle)

            angle = np.arctan2(vector2[1], vector2[0]) - np.arctan2(vector1[1], vector1[0])
            angle = np.degrees(angle)

            if angle > 180:
                angle -= 360
            elif angle < -180:
                angle += 360

            distance = np.linalg.norm(vector2)

            self.draw_lines_by_dots(frame, center_car, center_car_line1, center_circle)
            self.show_angle_and_distance(frame, angle, distance)

            self.send_data(angle, distance)
            # print(f"send_data --> Angle: {angle:.2f}, Distance: {distance:.2f}")
        else:
            pass

        return frame

    def run(self):
        try:
            while True:
                global counter, start_time
                # 每 100 次循环打印一次所需时间
                counter += 1
                if counter % 100 == 0:
                    end_time = time.time()
                    elapsed_time = end_time - start_time
                    print(f"100 次循环用时: {elapsed_time:.6f} 秒, 平均每次: {elapsed_time / 100:.6f} 秒")
                    # 重置开始时间
                    start_time = time.time()

                frame = self.camera.read_frame()
                frame = self.process_frame(frame)
                cv2.imshow('frame', frame)
                if cv2.waitKey(1) == ord('q'):
                    break
        finally:
            self.camera.release()
            self.esp32_communication.close()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    # 初始化计数器和开始时间
    counter = 0
    start_time = time.time()

    controller = Controller(
        camera_id=0,  # 摄像头 ID, 0, 1, 2... 逐个尝试一下
        aruco_id=30,  # 小车上的 arUco ID（https://chev.me/arucogen/）
        hsv_color=np.array([9, 190, 234]),  # 使用 get_hsv.py 获取
        param1=35,  # 边缘检测, 较低的值可以检测到更多的边缘
        param2=25,  # 圆形检测, 较低的值可以检测到更多的圆形
        min_dist=100,  # 圆心与圆心之间的最小距离圆心
        min_radius=10,  # 圆的最小半径
        max_radius=60,  # 圆的最大半径
        esp32_ip='192.168.3.190',  # esp32 IP，设置为 None 侧不发消息
        esp32_port=10000,  # esp32 端口，设置为 None 侧不发消息
        flask_ip='localhost',  # flask IP，设置为 None 侧不发消息
        flask_port=10001,  # flask 端口，设置为 None 侧不发消息
    )
    controller.run()
