import time
from pyaubo_agvc_sdk import RpcClient, Header
from pyaubo_agvc_sdk import AutoAlignRailwayStatus

# ====================== 初始化连接 ======================
agv = RpcClient()
agv.setRequestTimeout(1000)

ip = "127.0.0.1"
port = 30104
agv.connect(ip, port)
agv.login("admin", "admin")

print("\n===== 自动对齐轨道（异步接口） =====")

# 1. 抢夺控制权
agv.release_priority()
agv.set_priority("user", "")

# 2. 设置命令头
command_header = Header()
command_header.id = "auto_align_railway_test"

# 3. 调用异步接口：自动对齐轨道
agv.autoAlignRailway(command_header)

# 4. 循环等待异步执行完成
while True:
    # 获取异步状态
    async_status = agv.getAsyncInterfaceResultStatus()
    
    # 匹配命令ID
    if async_status.align_railway_status.header.id == command_header.id:
        # 判断是否完成
        if async_status.align_railway_status.status == AutoAlignRailwayStatus.FINISHED:

            ret = agv.autoAlignRailway(command_header)
            if ret == 10100000:
                print("自动对齐轨道完成")
                break
            else:
                print("自动对齐轨道失败，错误码", ret)
                break
    time.sleep(1)