import time
from pyaubo_agvc_sdk import RpcClient, Header, StationMark, Pose2d

# ====================== 初始化连接 ======================
agv = RpcClient()
agv.setRequestTimeout(1000)

ip = "127.0.0.1"
port = 30104
agv.connect(ip, port)
agv.login("admin", "admin")

print("\n====================")
agv.releasePriority()
agv.setPriority("user", "") # 抢占控制权

# 构造站点
stations = []
st = StationMark()
st.header = Header()
st.header.id = "add_station_1"
st.header.map_id = "current_map"
st.header.name = "ST1"
st.pose.x = 0
st.pose.y = 0
st.pose.yaw = 0
st.type = "ST"
stations.append(st)

# 返回结果
ret = agv.addStations(stations)
if ret == 10100000:
    print("添加站点成功")
else:
    print("添加站点失败，错误码", ret)