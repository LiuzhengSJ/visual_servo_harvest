import time
from pyaubo_agvc_sdk import RpcClient, Header
from pyaubo_agvc_sdk import SaveMapStatus

# 初始化连接
agv = RpcClient()
agv.setRequestTimeout(1000)

ip = "127.0.0.1"
port = 30104
agv.connect(ip, port)
agv.login("", "")

agv.releasePriority()
agv.setPriority("user", "")

# ====================== 保存地图 ======================
print("===== 开始保存地图 =====")
map_header = Header()
map_header.id = "save_map_test"
map_header.map_id = "map"
map_header.name = "test_map"

agv.saveMap(map_header)  # 修改

time.sleep(1)

# 等待保存完成
while True:
    mode_status = agv.getAsyncInterfaceResultStatus()
    if mode_status.save_map_status.header.id == map_header.id:
        if mode_status.save_map_status.status == SaveMapStatus.FINISHED:
            ret = agv.saveMap(map_header)
            if ret == 10100000:
                print("地图保存完成")
                break
            else:
                print("地图保存失败，错误码", ret)
                break
    time.sleep(1)