import time
from pyaubo_agvc_sdk import RpcClient

# ====================== 初始化连接 ======================
agv = RpcClient()
agv.setRequestTimeout(1000)

ip = "127.0.0.1"
port = 30104
agv.connect(ip, port)
agv.login("admin", "admin")

print("\n===== 实时前方障碍物检测 =====")

# 循环检测 10 次
for i in range(10):
    ret = agv.isObstacleAhead(detect_distance=1.0)
    
    if ret:
        print(f"第{i+1}次检测：前方有障碍物")
    else:
        print(f"第{i+1}次检测：前方无障碍物")
        
    time.sleep(1)