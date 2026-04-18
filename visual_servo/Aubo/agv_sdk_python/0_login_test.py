from pyaubo_agvc_sdk import RpcClient

"""创建并配置AGV客户端"""
client = RpcClient()
client.setRequestTimeout(1000)

# 连接到设备
ip = "127.0.0.1"
port = 30104
ret = client.connect(ip, port)
print(f"Connect return code: {ret}")

# 登录
login_result = client.login("admin", "admin")
print(f"Login result: {login_result}")