import json
from pprint import pprint

from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
import threading
import socket
import time

app = Flask(__name__)
socketio = SocketIO(app,
                    # async_mode='gevent',
                    cors_allowed_origins="*")
udp_thread = None
push_thread = None
thread_lock = threading.Lock()

# 全局变量
global_data = {"angle": [], "distance": []}

# 设置 pid 参数的端口，esp32 中设置, 发消息到 esp32
ESP32_PORT_PID = 10002
# 与 ESP32 中的 IFCONFIG[0] 一致
ESP32_IP = "192.168.3.190"

UDP_SERVER_PORT = 10001

UDP_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# 检查端口是否被占用
def is_port_in_use(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(('localhost', port)) == 0


if is_port_in_use(UDP_SERVER_PORT):
    print(f"UDP 服务器端口 {UDP_SERVER_PORT} 被占用, 程序退出")
    exit(1)


# 更新数据
def update_data(angle, distance):
    global global_data
    if len(global_data["angle"]) >= 100:
        global_data["angle"].pop(0)
        global_data["distance"].pop(0)
    global_data["angle"].append(angle)
    global_data["distance"].append(distance)


# UDP 服务函数
def udp_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_SERVER_PORT))
    print(f"UDP 服务器启动，监听端口 {UDP_SERVER_PORT}")

    while True:
        data, addr = sock.recvfrom(1024)  # 接收 UDP 数据
        message = json.loads(data.decode('utf-8'))
        # print(f"Received message: {message} from {addr}")

        # 1. 用于显示 angle 和 distance 的图表
        if "angle" in message:
            angle = int(float(message["angle"]))
            distance = int(float(message["distance"]))
            update_data(angle, distance)

        # 2. 用于显示 pid 参数的当前值
        elif "kp_angle" in message:
            # print('***###*************************************************')
            # pprint(message)
            if 'reset' in message:
                message.pop('reset')
            socketio.emit('show_cur_pid', message)
        else:
            pass


# 定时向前端推送数据
def push_data():
    global global_data
    while True:
        time.sleep(0.1)
        # print("angle:", global_data["angle"][-5:])
        # print("distance:", global_data["distance"][-5:])
        socketio.emit('update_chart_data', global_data)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/test')
def test():
    socketio.emit('show', {'message': 'Hello, World!'})
    return jsonify({'message': 'Hello, World!'})


@socketio.on('set_pid_param')
def handle_set_param(data):
    print("set_pid_param:", data)
    param = data.get('param')
    value = data.get('value')
    if not value:
        return
    # 使用 upd 发消息
    UDP_SOCKET.sendto(json.dumps({param: value}).encode('utf-8'), (ESP32_IP, ESP32_PORT_PID))


@socketio.on('connect')
def handle_connect():
    global udp_thread, push_thread, thread_lock
    with thread_lock:
        if udp_thread is None or not udp_thread.is_alive():
            print("启动 udp_thread 线程")
            udp_thread = threading.Thread(target=udp_server)
            udp_thread.daemon = True
            udp_thread.start()
    with thread_lock:
        if push_thread is None or not push_thread.is_alive():
            print("启动 push_thread 线程")
            push_thread = threading.Thread(target=push_data)
            push_thread.daemon = True
            push_thread.start()


if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=9999, debug=True)
