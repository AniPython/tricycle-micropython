
ip 地址说明：
192.168.3.190： esp32  ip
192.168.3.129： flask 服务器

端口号说明：  
9999： flask 服务器端口  
10000：opencv -->  esp32  作用：接收 angle 和 distance,  数据样例： {angle: 20, distance: 200}
10001：opencv -->  flask  作用：发送 angele 和 distance,  数据样例： {angle: 20, distance: 200}
10002：flask  -->  esp32  作用：更新 pid，数据样例： {kp_angle: 2}  
10001：esp32  -->  flask  作用：发送当前 pid 值，数据样例： {kp_angle: 2， kp_distance: 0.5 ...}

![安装制作](./imgs/make.png)
