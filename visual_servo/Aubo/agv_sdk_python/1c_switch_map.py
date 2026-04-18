import time
from pyaubo_agvc_sdk import RpcClient, Header
from pyaubo_agvc_sdk import SwitchMapStatus

# 初始化连接
agv = RpcClient()
agv.setRequestTimeout(1000)

ip = "127.0.0.1"
port = 30104
agv.connect(ip, port)
agv.login("", "")

agv.releasePriority()
agv.setPriority("user", "")

# ====================== 切换地图 ======================
print("===== 开始切换地图 =====")
map_header = Header()
map_header.id = "switch_map_test"
map_header.map_id = "map" # 切换到该id 的map 

agv.switchMap(map_header)

time.sleep(1)

# 等待切换完成
while True:
    mode_status = agv.getAsyncInterfaceResultStatus()
    if mode_status.switch_map_status.header.id == map_header.id:
        if mode_status.switch_map_status.status == SwitchMapStatus.FINISHED:
            ret = agv.switchMap(map_header)
            if ret == 10100000:
                print("地图切换完成")
                break
            else:
                print("地图切换失败，错误码", ret)
                break
    time.sleep(1)