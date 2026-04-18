# agvc_interface

接口说明文档网页链接：[接口类参考](http://agvc.pages.aubo-robotics.cn:8001/agvc_interface/classagvc__interface_1_1AgvcInterface.html)

## 2. python SDK的使用

### 2.1 Linux

安装l离线sdk pyaubo_agvc_sdk-0.8-cp310-cp310-manylinux2014_x86_64.whl
```
pip install *.whl --force-reinstall
```
查看是否安装：
```
pip list | grep pyaubo-agvc-sdk
```
如：
```
root@ros2-virtual-machine:~/# pip list | grep aubo
pyaubo-agvc-sdk                      0.8
```

**运行测试：**

简单结果测试：
```
python 0_login_test.py 
```
### 2.2 window版

编译

在agvc_interface目录下执行：
```
pip install *.whl
```

### 2.3 遇到问题

1. pip install 时网络不通

```
pip install *.whl 时遇到下面问题，是因为需要更换源安装 pybind11

WARNING: Retrying (Retry(total=4, connect=None, read=None, redirect=None, status=None)) after connection broken by 'SSLError(SSLEOFError(8, '[SSL: UNEXPECTED_EOF_WHILE_READING] EOF occurred in violation of protocol (_ssl.c:1007)'))': /simple/pybind11/
WARNING: Retrying (Retry(total=3, connect=None, read=None, redirect=None, status=None)) after connection broken by 'SSLError(SSLEOFError(8, '[SSL: UNEXPECTED_EOF_WHILE_READING] EOF occurred in violation of protocol (_ssl.c:1007)'))': /simple/pybind11/
WARNING: Retrying (Retry(total=2, connect=None, read=None, redirect=None, status=None)) after connection broken by 'SSLError(SSLEOFError(8, '[SSL: UNEXPECTED_EOF_WHILE_READING] EOF occurred in violation of protocol (_ssl.c:1007)'))': /simple/pybind11/
WARNING: Retrying (Retry(total=1, connect=None, read=None, redirect=None, status=None)) after connection broken by 'SSLError(SSLEOFError(8, '[SSL: UNEXPECTED_EOF_WHILE_READING] EOF occurred in violation of protocol (_ssl.c:1007)'))': /simple/pybind11/
WARNING: Retrying (Retry(total=0, connect=None, read=None, redirect=None, status=None)) after connection broken by 'SSLError(SSLEOFError(8, '[SSL: UNEXPECTED_EOF_WHILE_READING] EOF occurred in violation of protocol (_ssl.c:1007)'))': /simple/pybind11/
Could not fetch URL https://pypi.org/simple/pybind11/: There was a problem confirming the ssl certificate: HTTPSConnectionPool(host='pypi.org', port=443): Max retries exceeded with url: /simple/pybind11/ (Caused by SSLError(SSLEOFError(8, '[SSL: UNEXPECTED_EOF_WHILE_READING] EOF occurred in violation of protocol (_ssl.c:1007)'))) - skipping
```

解决： 

```
 pip install *.whl --force-reinstall -i https://pypi.tuna.tsinghua.edu.cn/simple
```

