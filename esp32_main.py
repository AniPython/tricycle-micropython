import socket
from machine import PWM, Pin
import network
import time
import ujson
import _thread

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
MOTORS = {
    "left": [PWM(Pin(IN1), freq=1000), PWM(Pin(IN2), freq=1000)],
    "right": [PWM(Pin(IN3), freq=1000), PWM(Pin(IN4), freq=1000)],
}


# PID 控制参数
Kp_dist = 0.5
Ki_dist = 0.001
Kd_dist = 0.08

Kp_angle = 3
Ki_angle = 0.03
Kd_angle = 2

# 上次循环误差
pre_error_dist = 0
pre_error_angle = 0

# 积累误差
integral_dist = 0
integral_angle = 0

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
    global pre_error_dist, integral_dist, pre_error_angle, integral_angle
    for motor in MOTORS.values():
        for pwm in motor:
            pwm.duty(0)
    pre_error_dist = 0
    integral_dist = 0
    pre_error_angle = 0
    integral_angle = 0
    print("Motor stopped!")


def limit_value(value, min_value=0, max_value=1023):
    return max(min_value, min(max_value, value))

def set_motor_speed(left_speed, right_speed):
    def set_pwm(pwm1, pwm2, speed):
        if speed > 0:
            pwm1.duty(0)
            pwm2.duty(limit_value(abs(speed) + MOTOR_MIN_DUTY))
        else:
            pwm1.duty(limit_value(abs(speed) + MOTOR_MIN_DUTY))
            pwm2.duty(0)

    set_pwm(*MOTORS["left"], left_speed)
    set_pwm(*MOTORS["right"], right_speed)


# PID 控制
def pid_control(angle, distance):
    global pre_error_dist, integral_dist, pre_error_angle, integral_angle

    # 计算距离误差
    error_dist = distance
    integral_dist += error_dist
    diff_dist = error_dist - pre_error_dist
    pre_error_dist = error_dist

    # 计算角度误差
    error_angle = angle
    integral_angle += error_angle
    diff_angle = error_angle - pre_error_angle
    pre_error_angle = error_angle

    # PID 控制计算
    control_signal_dist = Kp_dist * error_dist + Ki_dist * integral_dist + Kd_dist * diff_dist
    control_signal_angle = Kp_angle * error_angle + Ki_angle * integral_angle + Kd_angle * diff_angle

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
        data = conn_data.recv(1024)
        if not data:
            break
        try:
            command = data.decode().strip()

            # 没有检测到目标, 暂停电机
            if command == "0:0":
                stop_motor()
                conn_data.sendall(b'OK\n')
                continue

            # 解析角度和距离数据
            angle, distance = map(float, command.split(':'))
            # print(f"Received angle: {angle}, distance: {distance}")

            # 判断如果完成任务, 暂停电机
            if abs(angle) < SUCCESS_ANGLE and abs(distance) < SUCCESS_DISTANCE:
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
    global Kp_dist, Ki_dist, Kd_dist, Kp_angle, Ki_angle, Kd_angle
    data = conn_pid.recv(1024)
    try:
        params = ujson.loads(data.decode().strip())
        if 'Kp_dist' in params:
            Kp_dist = params['Kp_dist']
        if 'Ki_dist' in params:
            Ki_dist = params['Ki_dist']
        if 'Kd_dist' in params:
            Kd_dist = params['Kd_dist']
        if 'Kp_angle' in params:
            Kp_angle = params['Kp_angle']
        if 'Ki_angle' in params:
            Ki_angle = params['Ki_angle']
        if 'Kd_angle' in params:
            Kd_angle = params['Kd_angle']
        print(f"Updated PID parameters:\n{Kp_dist=}\n{Ki_dist=}\n{Kd_dist=}\n{Kp_angle=}\n{Ki_angle=}\n{Kd_angle=}\n")
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
