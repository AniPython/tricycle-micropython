import json
import socket
from pprint import pprint
from collections import OrderedDict

ESP32_IP = '192.168.3.190'  # 修改为你的 ESP32 的 IP 地址
ESP32_PORT = 12346  # 修改为你的 ESP32 上的端口号

# PID 控制参数
para_dict = OrderedDict(
    Kp_dist=0.6,
    Ki_dist=0.001,
    Kd_dist=0.08,

    Kp_angle=3,
    Ki_angle=0.03,
    Kd_angle=2,
)
json_str = json.dumps(para_dict) + '\n'
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((ESP32_IP, ESP32_PORT))
sock.sendall(json_str.encode())
if sock.recv(3) == b'OK\n':
    print('PID 参数设置成功')
    pprint(para_dict)
else:
    print('PID 参数设置失败')
sock.close()
print('finished')

# # PID 控制参数
# Kp_dist = 0.2
# Ki_dist = 0.001
# Kd_dist = 0.08
#
# Kp_angle = 4
# Ki_angle = 0.02
# Kd_angle = 1
