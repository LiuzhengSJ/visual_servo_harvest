import time
from pyaubo_agvc_sdk import RpcClient, Header, NavGoalType, NavType, NavStatus, Speed

# ====================== 初始化连接 ======================
agv = RpcClient()
agv.setRequestTimeout(1000)

ip = "127.0.0.1"
port = 30104
agv.connect(ip, port)
agv.login("admin", "admin")

# ====================== 导航任务测试 ======================
print("\n===== 开始导航任务测试 =====")

# 1. 抢夺控制权
agv.releasePriority()
agv.setPriority("user", "") 

# ==========================================
# 2. 设置第一段导航任务
# ==========================================
nav_goal = NavGoalType()
nav_goal.header = Header()
nav_goal.header.id = "test_nav_goal_1"
nav_goal.header.map_id = "current_map"
nav_goal.type = NavType.PATH_TO_STATION
nav_goal.forward_flag = False
nav_goal.goal_end_rotate = False
nav_goal.max_vel = Speed()
nav_goal.max_vel.v = 0.5
nav_goal.stations_id = ["add_station_2", "add_station_1"]

# 3. 下发第一段导航
ret = agv.setNavGoal(nav_goal)
if ret == 10100000:
    print("下发导航任务成功")
else:
    print(f"下发导航任务失败: {ret}")
    exit(1)
time.sleep(1)

# 4. 循环判断任务状态
while True:
    nav_info = agv.getNavInfo()
    
    # 确认任务ID
    if nav_info.header.id != "test_nav_goal_1":
        time.sleep(1)
        continue

    # 状态判断
    if nav_info.status == NavStatus.FINISHED:
        print("nav back finish!")
        break
    elif nav_info.status == NavStatus.FAILED:
        print("nav back failed!")
        break
    elif nav_info.status == NavStatus.RUNNING:
        print("nav back running!")
    
    time.sleep(1)

time.sleep(1)

# ==========================================
# 5. 设置第二段导航任务
# ==========================================
nav_goal.header.id = "test_nav_goal_2"
nav_goal.header.map_id = "current_map"
nav_goal.type = NavType.PATH_TO_STATION
nav_goal.forward_flag = True
nav_goal.goal_end_rotate = False
nav_goal.max_vel.v = 0.5
nav_goal.stations_id = ["add_station_1", "add_station_2"]

# 6. 下发第二段导航
ret = agv.setNavGoal(nav_goal)
if ret == 10100000:
    print("下发导航任务成功")
else:
    print(f"下发导航任务失败: {ret}")
    exit(1)
time.sleep(1)

# 7. 循环判断任务状态
while True:
    nav_info = agv.getNavInfo()
    
    if nav_info.header.id != "test_nav_goal_2":
        time.sleep(1)
        continue

    if nav_info.status == NavStatus.FINISHED:
        print("nav finish!")
        break
    elif nav_info.status == NavStatus.FAILED:
        print("nav failed!")
        break
    elif nav_info.status == NavStatus.RUNNING:
        print("nav running!")
    
    time.sleep(1)

print("\n 导航任务全部执行完成")