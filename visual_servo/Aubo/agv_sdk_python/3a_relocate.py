import time
from pyaubo_agvc_sdk import RpcClient, Pose2d, Header, ChangeRunningModeStatus

# ====================== 初始化连接 ======================
agv = RpcClient()
agv.setRequestTimeout(1000)

ip = "127.0.0.1"
port = 30104
agv.connect(ip, port)
agv.login("admin", "admin")

# ====================== 重定位测试 ======================
print("\n===== 开始重定位测试 =====")

# 1. 抢夺控制权
agv.releasePriority()
agv.setPriority("user", "")

# 2. 获取运行信息，查看定位是否丢失
running_info = agv.getRunningInfo()

# 3. 如果定位没有丢失，执行重定位
if not running_info.localization_loss:
    print("当前定位正常，开始执行重定位...")

    # 3.1 设置重定位位姿 & 命令头
    init_pose = Pose2d()
    init_pose.x = 0.0
    init_pose.y = 0.0
    init_pose.yaw = 0.0

    command_header = Header()
    command_header.id = "test_relocation"

    # 3.2 调用重定位接口
    agv.relocation(init_pose, command_header)

    # 3.3 循环等待重定位完成
    while True:
        mode_status = agv.getAsyncInterfaceResultStatus()
        
        # 确认命令ID一致
        if mode_status.relocation_status.header.id == command_header.id:
            # 判断状态是否完成
            if mode_status.relocation_status.status == ChangeRunningModeStatus.FINISHED:
                # 再次调用接口确认结果
                ret = agv.relocation(init_pose, command_header)
                if ret == 10100000:
                    print("重定位成功")
                else:
                    print("重定位失败，错误码:", ret)
                break
            else:
                print("重定位中")
        time.sleep(1)
else:
    print("定位已丢失，无法执行重定位")