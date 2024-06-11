import socket
from machine import PWM, Pin
import network
import time
import ujson
import _thread
from collections import OrderedDict

# ######################################################
# 配置全局变量开始
# ######################################################
SSID = 'Yi'
PASSWORD = '88889999'
IFCONFIG = ("192.168.3.190", "255.255.255.0", "192.168.3.1", "8.8.8.8")
PORT_DATA = 12345
PORT_PID = 12346

# 启动电机最小 duty 值
MOTOR_MIN_DUTY = 120

# 成功角度和距离，同时满足条件，电机停止
SUCCESS_ANGLE = 10
SUCCESS_DISTANCE = 60

# 电机控制引脚
IN1, IN2, IN3, IN4 = 12, 14, 27, 26

# 初始化电机 PWM
LEFT_PWM1 = PWM(Pin(IN1), freq=1000)
LEFT_PWM2 = PWM(Pin(IN2), freq=1000)
RIGHT_PWM1 = PWM(Pin(IN3), freq=1000)
RIGHT_PWM2 = PWM(Pin(IN4), freq=1000)

# PID 控制参数
pid_data = OrderedDict([
    # PID 控制参数
    # 1. 距离
    ("kp_dist", 0.5),
    ("ki_dist", 0.001),
    ("kd_dist", 0.08),
    # 2. 角度
    ("kp_angle", 3),
    ("ki_angle", 0.03),
    ("kd_angle", 2),

    # 上次循环误差
    ("pre_error_dist", 0),
    ("pre_error_angle", 0),

    # 积累误差
    ("integral_dist", 0),
    ("integral_angle", 0),

    # 目标值
    ("target_angle", 0),
    ("target_distance", 150)
])


# ######################################################
# 配置全局变量结束
# ######################################################


# 连接 WiFi
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.ifconfig(IFCONFIG)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    print('Connecting to WiFi...', end='')
    while not wlan.isconnected():
        time.sleep(0.5)
        print('.', end='')
    print('\nConnection successful')
    print(wlan.ifconfig())


def stop_motor():

    LEFT_PWM1.duty(0)
    LEFT_PWM2.duty(0)
    RIGHT_PWM1.duty(0)
    RIGHT_PWM2.duty(0)

    pid_data["pre_error_dist"] = 0
    pid_data["integral_dist"] = 0
    pid_data["pre_error_angle"] = 0
    pid_data["integral_angle"] = 0

    print("Motor stopped!")


def limit_value(value, min_value=0, max_value=1023):
    return max(min_value, min(max_value, value))


def set_motor_speed(left_speed, right_speed):

    if left_speed > 0:
        LEFT_PWM1.duty(0)
        LEFT_PWM2.duty(limit_value(abs(left_speed) + MOTOR_MIN_DUTY))
    else:
        LEFT_PWM1.duty(limit_value(abs(left_speed) + MOTOR_MIN_DUTY))
        LEFT_PWM2.duty(0)

    if right_speed > 0:
        RIGHT_PWM1.duty(0)
        RIGHT_PWM2.duty(limit_value(abs(right_speed) + MOTOR_MIN_DUTY))
    else:
        RIGHT_PWM1.duty(limit_value(abs(right_speed) + MOTOR_MIN_DUTY))
        RIGHT_PWM2.duty(0)


# PID 控制
def pid_control(angle, distance):
    global pid_data

    # 计算距离误差
    error_dist = distance - pid_data["target_distance"]
    pid_data["integral_dist"] += error_dist
    diff_dist = error_dist - pid_data["pre_error_dist"]
    pid_data["pre_error_dist"] = error_dist

    # 计算角度误差
    error_angle = angle - pid_data["target_angle"]
    pid_data["integral_angle"] += error_angle
    diff_angle = error_angle - pid_data["pre_error_angle"]
    pid_data["pre_error_angle"] = error_angle

    # PID 控制计算
    control_signal_dist = (
            pid_data["kp_dist"] * error_dist +
            pid_data["ki_dist"] * pid_data["integral_dist"] +
            pid_data["kd_dist"] * diff_dist
    )
    control_signal_angle = (
            pid_data["kp_angle"] * error_angle +
            pid_data["ki_angle"] * pid_data["integral_angle"] +
            pid_data["kd_angle"] * diff_angle
    )

    # 先忽略角度，测试两个轮子转动方向是否正确，
    # 如果左轮不正确， 调换 MOTORS 的 IN1 和 IN2 位置
    # 如果右轮不正确， 调换 MOTORS 的 IN3 和 IN4 位置
    # control_signal_angle = 0

    # 计算左右轮速度
    left_speed = int(control_signal_dist + control_signal_angle)
    right_speed = int(control_signal_dist - control_signal_angle)

    return left_speed, right_speed


# 数据连接处理
def handle_data_connection(conn_data):

    while True:
        # 接收数据
        data = conn_data.recv(1024)
        if not data:
            break
        try:
            command = data.decode().strip()

            # 没有检测到目标, 暂停电机
            if command == "0:0":
                # print("No target detected, stop motor")
                stop_motor()
                conn_data.sendall(b'OK\n')
                continue

            # 解析角度和距离数据
            angle, distance = map(float, command.split(':'))
            # print(f"Received angle: {angle}, distance: {distance}")

            # 判断如果完成任务, 暂停电机
            if abs(angle) < SUCCESS_ANGLE and abs(distance) < SUCCESS_DISTANCE:
                # print("Mission completed, stop motor")
                stop_motor()
                conn_data.sendall(b'OK\n')
                continue

            # 利用角度和距离数据, PID方式控制电机
            left_speed, right_speed = pid_control(angle, distance)
            # print(f"Left speed: {left_speed}, Right speed: {right_speed}")
            set_motor_speed(left_speed, right_speed)

            # 给客户端返回确认消息, 让客户端进入下一个循环, 不断发送新数据
            conn_data.sendall(b'OK\n')
        except Exception as e:
            # print(f"Error: {e}")
            stop_motor()
            conn_data.sendall(b'OK\n')

    conn_data.close()


def handle_pid_connection(conn_pid):
    global pid_data
    data = conn_pid.recv(1024)
    try:
        params = ujson.loads(data.decode().strip())
        pid_data.update(params)
        print(f"Updated PID parameters: \n{pid_data}\n")
        conn_pid.sendall(b'OK\n')
    except Exception as e:
        print(f"Error: {e}")
        conn_pid.sendall(b'er\n')
    finally:
        conn_pid.close()


# 启动线程处理不同连接
def start_data_thread():
    sock_data = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_data.bind(('', PORT_DATA))
    sock_data.listen(1)
    print(f'Socket data listening on port {PORT_DATA}')
    while True:
        conn_data, addr_data = sock_data.accept()
        print('Connected by data sender:', addr_data)
        handle_data_connection(conn_data)


def start_pid_thread():
    sock_pid = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_pid.bind(('', PORT_PID))
    sock_pid.listen(1)
    print(f'Socket pid listening on port {PORT_PID}')
    while True:
        conn_pid, addr_pid = sock_pid.accept()
        print('Connected by PID parameter sender:', addr_pid)
        handle_pid_connection(conn_pid)


if __name__ == '__main__':

    # 连接 WiFi
    connect_wifi()

    # 启动线程
    _thread.start_new_thread(start_data_thread, ())
    _thread.start_new_thread(start_pid_thread, ())

    # 主线程
    while True:
        time.sleep(1)
