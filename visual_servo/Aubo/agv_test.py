from pyaubo_agvc_sdk import RpcClient
import time


def main():

    AGV_IP = '192.168.192.100'
    AGV_PORT = 30104

    agv = RpcClient()
    agv.setRequestTimeout(1000)
    print(f"\n正在连接AGV | IP: {AGV_IP}:{AGV_PORT}")

    connect_ret = agv.connect(AGV_IP, AGV_PORT)
    if connect_ret != 0:
        print(f"AGV连接失败（返回码{connect_ret}），程序退出")
        return

    login_ret = agv.login("admin", "admin")
    if login_ret != 0:
        print(f"AGV登录失败（返回码{login_ret}），程序退出")
        agv.disconnect()
    print("AGV连接与登录成功")
    t=1
    time.sleep(t)
    v=0.2

    # login_result = agv.login("admin", "admin")
    # print(f"Login result: {login_result}")

    agv.release_priority()
    time.sleep(0.5)
    control_ret = agv.set_priority("admin", AGV_IP)

    if control_ret != 10100000:
        print(f"AGV获取控制权失败（返回码{control_ret}），程序退出")
        agv.disconnect()
        return
    print("AGV控制权获取成功")


    agv.set_control_speed(v, 0.0)
    time.sleep(t)
    agv.set_control_speed(0.0, 0.0)
    time.sleep(t)

    agv.set_control_speed(-v, 0.0)
    time.sleep(t)
    agv.set_control_speed(0.0, 0.0)
    time.sleep(t)

    agv.disconnect()


if __name__ == "__main__":
    main()