import time
from pyaubo_agvc_sdk import RpcClient, RunningMode, Header, Speed
from pyaubo_agvc_sdk import ChangeRunningModeStatus

# 初始化连接
agv = RpcClient()
agv.setRequestTimeout(1000)

ip = "127.0.0.1"
port = 30104
agv.connect(ip, port)
agv.login("admin", "admin")

# ====================== 切换到建图模式 ======================
print("===== 开始切换建图模式 =====")
agv.releasePriority()
agv.setPriority("user", "")

running_mode = RunningMode.MAPPING
command_header = Header()
command_header.id = "create_map"

agv.changeRunningMode(running_mode, command_header)

time.sleep(1)

# 等待切换完成
while True:
    mode_status = agv.getAsyncInterfaceResultStatus()
    if mode_status.change_running_mode_status.header.id == command_header.id:
        if mode_status.change_running_mode_status.status == ChangeRunningModeStatus.FINISHED:
            ret = agv.changeRunningMode(running_mode, command_header)
            if ret == 10100000:
                print("已进入建图模式")
                break
            else:
                print("未进入建图模式，错误码", ret)
                break
    time.sleep(1)


# ====================== 开环控制建图 ======================
print("===== 开始开环扫描地图 =====")
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
print("开环扫描完成")
