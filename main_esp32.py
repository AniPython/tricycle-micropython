import socket
from machine import PWM, Pin
import network
import time
import ujson
import _thread
from collections import OrderedDict


a_lock = _thread.allocate_lock()

run_data = {
    'angle': 0,
    'distance': 0
}

last_run_data = {
    'angle': 0,
    'distance': 0
}

class WiFiManager:
    def __init__(self, ssid, password, ifconfig):
        self.ssid = ssid
        self.password = password
        self.ifconfig = ifconfig

    def connect(self):
        wlan = network.WLAN(network.STA_IF)
        wlan.ifconfig(self.ifconfig)
        wlan.active(True)
        wlan.connect(self.ssid, self.password)
        print('Connecting to WiFi...', end='')
        while not wlan.isconnected():
            time.sleep(0.5)
            print('.', end='')
        print('\nConnection successful')
        print(wlan.ifconfig())


class MotorController:
    def __init__(self, in1, in2, in3, in4, min_duty=120, reverse_left=False, reverse_right=False):
        self.min_duty = min_duty

        if reverse_left:
            self.left_pwm1 = PWM(Pin(in2), freq=1000)
            self.left_pwm2 = PWM(Pin(in1), freq=1000)
        else:
            self.left_pwm1 = PWM(Pin(in1), freq=1000)
            self.left_pwm2 = PWM(Pin(in2), freq=1000)

        if reverse_right:
            self.right_pwm1 = PWM(Pin(in4), freq=1000)
            self.right_pwm2 = PWM(Pin(in3), freq=1000)
        else:
            self.right_pwm1 = PWM(Pin(in3), freq=1000)
            self.right_pwm2 = PWM(Pin(in4), freq=1000)

    def stop(self):
        self.left_pwm1.duty(0)
        self.left_pwm2.duty(0)
        self.right_pwm1.duty(0)
        self.right_pwm2.duty(0)
        time.sleep(0.6)
        print("Motor stopped!")

    def limit_value(self, value, min_value=0, max_value=1023):
        return max(min_value, min(max_value, value))

    def set_speed(self, left_speed, right_speed):
        if left_speed > 0:
            self.left_pwm1.duty(0)
            self.left_pwm2.duty(self.limit_value(abs(left_speed) + self.min_duty))
        else:
            self.left_pwm1.duty(self.limit_value(abs(left_speed) + self.min_duty))
            self.left_pwm2.duty(0)

        if right_speed > 0:
            self.right_pwm1.duty(0)
            self.right_pwm2.duty(self.limit_value(abs(right_speed) + self.min_duty))
        else:
            self.right_pwm1.duty(self.limit_value(abs(right_speed) + self.min_duty))
            self.right_pwm2.duty(0)


class ServoController:
    def __init__(self, pin, start_angle, amp_angle, max_angle=270):
        self.servo_pwm = PWM(Pin(pin), freq=50) if pin is not None else None
        self.start_angle = start_angle
        self.amp_angle = amp_angle
        self.max_angle = max_angle
        if self.servo_pwm:
            self.set_angle(self.start_angle)
            self.servo_pwm.duty(0)

    def set_angle(self, angle):
        if not self.servo_pwm:
            return
        min_duty = 1024 * 0.025
        max_duty = 1024 * 0.125
        duty = int(min_duty + (angle / self.max_angle) * (max_duty - min_duty))
        self.servo_pwm.duty(duty)

    def swing(self):
        if self.servo_pwm:
            self.set_angle(self.start_angle)
            time.sleep(0.5)
            self.set_angle(self.start_angle + self.amp_angle)
            time.sleep(0.8)
            self.set_angle(self.start_angle)
            time.sleep(0.5)
            self.servo_pwm.duty(0)


class PIDController:
    def __init__(self, initial_data):

        self.pid_data = OrderedDict(initial_data)
        self.pid_data_default = self.pid_data.copy()

        self.pre_error_distance = 0
        self.pre_error_angle = 0
        self.integral_distance = 0
        self.integral_angle = 0

    def update_params(self, params):
        self.pid_data.update(params)
        print(f"Updated PID parameters: \n{self.pid_data}\n")

    def set_default(self):
        self.pid_data = self.pid_data_default.copy()
        print(f"Set PID parameters to default: \n{self.pid_data}\n")

    def zero(self):
        self.integral_distance = 0
        self.integral_angle = 0
        self.pre_error_distance = 0
        self.pre_error_angle = 0

    def calculate(self, angle, distance):
        # Calculate distance error
        error_distance = distance - self.pid_data["target_distance"]
        self.integral_distance += error_distance
        diff_distance = error_distance - self.pre_error_distance
        self.pre_error_distance = error_distance

        # Calculate angle error
        error_angle = angle - self.pid_data["target_angle"]
        self.integral_angle += error_angle
        diff_angle = error_angle - self.pre_error_angle
        self.pre_error_angle = error_angle

        # PID control calculations
        control_signal_distance = (
                self.pid_data["kp_distance"] * error_distance +
                self.pid_data["ki_distance"] * self.integral_distance +
                self.pid_data["kd_distance"] * diff_distance
        )
        control_signal_angle = (
                self.pid_data["kp_angle"] * error_angle +
                self.pid_data["ki_angle"] * self.integral_angle +
                self.pid_data["kd_angle"] * diff_angle
        )

        # Calculate left and right wheel speeds
        left_speed = int(control_signal_distance + control_signal_angle)
        right_speed = int(control_signal_distance - control_signal_angle)

        return left_speed, right_speed


class ControlHandler:
    def __init__(self, motor_controller, servo_controller, pid_controller, swing_angle, swing_distance):
        self.motor_controller = motor_controller
        self.servo_controller = servo_controller
        self.pid_controller = pid_controller
        self.swing_angle = swing_angle
        self.swing_distance = swing_distance

    def run(self):
        global run_data, last_run_data
        PWM(Pin(2, Pin.OUT)).duty(512)
        # data_dict = ujson.loads(data.decode())
        # angle = data_dict.get('angle')
        # distance = data_dict.get('distance')
        while True:

            # 判断 run_data 有没有发生变化
            angle = run_data['angle']
            distance = run_data['distance']

            if (angle == last_run_data['angle']) and (distance == last_run_data['distance']):
                continue
            else:
                last_run_data['angle'] = angle
                last_run_data['distance'] = distance

            # if angle == run_data['angle'] and distance == run_data['distance']:
            #     continue

            if angle == 0 and distance == 0:
                self.motor_controller.stop()
                self.pid_controller.zero()
                time.sleep(0.6)
                continue

            if abs(angle) < self.swing_angle and abs(distance) < self.swing_distance:
                self.motor_controller.stop()
                self.pid_controller.zero()
                self.servo_controller.swing()
                time.sleep(0.6)
                continue

            left_speed, right_speed = self.pid_controller.calculate(angle, distance)
            self.motor_controller.set_speed(left_speed, right_speed)


# class SetPIDHandler:
#     def __init__(self, pid_controller):
#         self.pid_controller = pid_controller
#
#     def handle_connection(self, data: bytes):
#         try:
#             print("handle_connection")
#             params = ujson.loads(data.decode().strip())
#             print(params)
#             if 'reset' in params:
#                 print("Reset PID parameters to default")
#                 self.pid_controller.set_default()
#             else:
#                 self.pid_controller.update_params(params)
#         except Exception as e:
#             print(str(e))


def start_udp_recv_data_thread(controller, port):
    global run_data
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    print(f'Socket receive control data listening on port {port}')
    while True:
        data, addr = sock.recvfrom(1024)
        params = ujson.loads(data.decode())

        if 'angle' in params:
            run_data['angle'] = params['angle']
            run_data['distance'] = params['distance']
        elif 'reset' in params:
            print("Reset PID parameters to default")
            controller.pid_controller.set_default()
        else:
            controller.pid_controller.update_params(params)
        # handler.handle_connection(data)

# def start_udp_recv_set_pid_thread(controller, ip, port):
#     pass

def start_send_cur_pid_thread(controller, ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        sock.sendto(ujson.dumps(controller.pid_data).encode(), (ip, port))
        time.sleep(1)


def blink():
    led_pwm = PWM(Pin(2, Pin.OUT))
    while True:
        led_pwm.duty(512)
        time.sleep(0.5)
        led_pwm.duty(0)
        time.sleep(0.5)


if __name__ == '__main__':
    # 配置参数
    SSID = 'Yi'
    PASSWORD = '88889999'
    IFCONFIG = ("192.168.3.190", "255.255.255.0", "192.168.3.1", "8.8.8.8")
    PORT_DATA = 10000  # 收 控制指令
    PORT_PID = 10002  # 收 pid 参数用于设置
    PC_UDP_SERVER_PROT = 10001  # 发 pid 参数用于显示
    PC_IP = "192.168.3.129"

    # PID 控制参数初始化
    pid_initial_data = [
        ("kp_distance", 0.5),
        ("ki_distance", 0.01),
        ("kd_distance", 0.08),
        ("kp_angle", 3),
        ("ki_angle", 0.03),
        ("kd_angle", 2),
        ("target_angle", 0),
        ("target_distance", 130)
    ]

    # 创建对象
    wifi_manager = WiFiManager(SSID, PASSWORD, IFCONFIG)
    motor_controller = MotorController(13, 12, 14, 27, reverse_left=True, reverse_right=True)
    servo_controller = ServoController(26, start_angle=10, amp_angle=80, max_angle=270)
    pid_controller = PIDController(pid_initial_data)
    data_handler = ControlHandler(motor_controller, servo_controller, pid_controller, swing_angle=10, swing_distance=150)
    # set_pid_handler = SetPIDHandler(pid_controller)

    # 连接 WiFi
    wifi_manager.connect()

    # 启动线程
    # 1. 接收 angle 和 distance
    _thread.start_new_thread(start_udp_recv_data_thread, (pid_controller, PORT_DATA))
    # 2. 接收 PID 参数进行设置
    # _thread.start_new_thread(start_udp_recv_thread, (set_pid_handler, PORT_PID))
    # 3. 发送当前 PID 参数
    _thread.start_new_thread(start_send_cur_pid_thread, (pid_controller, PC_IP, PC_UDP_SERVER_PROT))

    # 主线程
    # blink()
    data_handler.run()


