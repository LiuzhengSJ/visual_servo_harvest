import time
from pyaubo_agvc_sdk import RpcClient, Header, PathStation, PathShape

# ====================== 初始化连接 ======================
agv = RpcClient()
agv.setRequestTimeout(1000)

ip = "127.0.0.1"
port = 30104
agv.connect(ip, port)
agv.login("admin", "admin")

print("\n====================")
agv.releasePriority()
agv.setPriority("user", "")  # 抢占控制权

# 构造路径
path = PathStation()
path.header = Header()
path.header.id = "new_path_1-2"
path.header.map_id = "current_map"
path.header.name = "path1-2"
path.start_station_id = "add_station_1"
path.end_station_id = "add_station_2"
path.shape = PathShape.LINE
path.use_direction = False
path.max_speed = 1.0

# 返回结果
ret = agv.generatePath(path)
if ret == 10100000:
    print("生成路径成功")
else:
    print("生成路径失败，错误码", ret)