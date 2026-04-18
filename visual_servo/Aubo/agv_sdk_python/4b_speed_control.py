import time
from pyaubo_agvc_sdk import RpcClient, Header, NavGoalType, NavType, NavStatus, Speed

# ====================== 初始化连接 ======================
agv = RpcClient()
agv.setRequestTimeout(1000)

ip = "127.0.0.1"
port = 30104
agv.connect(ip, port)
agv.login("admin", "admin")

# ====================== 开环控制测试 ======================
print("\n===== 开环控制测试 =====")

# 1. 抢夺控制权
agv.releasePriority()
agv.setPriority("user", "") 

# 前进2秒
start = time.time()
while time.time() - start < 2:
    speed = Speed()
    speed.v = 0.5
    speed.w = 0.0
    agv.setControlSpeed(speed)
    time.sleep(0.1)

# 旋转2秒
start = time.time()
while time.time() - start < 2:
    speed = Speed()
    speed.v = 0.0
    speed.w = 0.5
    agv.setControlSpeed(speed)
    time.sleep(0.1)

speed = Speed()
speed.v = 0.0
speed.w = 0.0
agv.setControlSpeed(speed)
print("开环控制完成")