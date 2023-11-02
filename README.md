# xCore SDK 机器人控制接口

* xCore SDK编程接口库是珞石机器人提供给客户用于二次开发的软件产品，通过编程接口库，客户可以对机器人进行一系列控制和操作。
* xCore SDK提供对机器人的非实时控制和实时控制。
  * 非实时控制主要通过发送目标点为关节点位或笛卡尔点位的运动指令，使用控制器内部的轨迹规划，完成如MoveL，MoveJ等运动。
此外，还支持查询机器人状态、位姿信息、DI/DO控制等操作。
  * 实时模式最高达1KHz的实时控制，可用于算法验证以及新应用的开发。

## 兼容性
### 机器人控制器
xCore控制器版本 v2.1.0+
### 编译环境
|操作平台|编译器| 平台                 |语言|
|----|---|--------------------|----|
|Ubuntu 18.04/20.04/22.04|build-essential| x86_64</br>aarch64 |C++|
|Windows 10|MSVC 14.1+| x86_64             |C++|

## 编译
C++版本xCore SDK使用CMake构建工程，CMake版本不低于3.12。

### 准备工作
#### Ubuntu
* 安装g++和cmake `sudo apt install cmake g++`
* 对于Ubuntu 18.04，默认的CMake版本是3.10，可使用下列方法安装最新版本的CMake
~~~
sudo apt remove --purge --auto-remove cmake
sudo apt update && \
sudo apt install -y software-properties-common lsb-release && \
sudo apt clean all
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
sudo apt update
sudo apt install kitware-archive-keyring
sudo rm /etc/apt/trusted.gpg.d/kitware.gpg
~~~
如果运行 `sudo apt update`出现`NO_PUBKEY`错误，运行:
~~~
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 6AF7F09730B3F0A4
~~~
再继续执行：
~~~
sudo apt update
sudo apt install cmake
~~~

#### Windows
* 下载并安装Microsoft Visual Studio，版本不低于2017，选择安装 *使用C++的桌面应用* 。

### 编译Targets

*Note:* 根据不同的可执行程序名称，编译目标名称也会不同

* Build
    * *all* (the default if no target is provided)
    * *clean*
    * *sdk_example* - 示例程序
    * *install* - 安装可执行文件到 *CMAKE_INSTALL_PREFIX*
    * *doc* - 生成API文档, 需要安装Doxygen

### CMake选项

* `CMAKE_INSTALL_PREFIX` - 安装路径
* `CMAKE_BUILD_TYPE` - 编译类型 
* `XCORE_LINK_SHARED_LIBS` - 是否链接动态库
* `XCORE_USE_XMATE_MODEL` - 是否使用xMate模型库进行运动学和动力学计算。目前支持Linux x86_64和Windows 64bit。 

## 使用方法
### 硬件设置
xCore SDK通过以太网（TCP/IP）连接机器人。如果只使用非实时控制，对网络的要求并不高，通过有线或无线连接皆可，使工作站PC和机器人连接同一局域网。
如果使用实时模式，建议通过有线网直连机器人，以保证网络稳定性。
### 机器人设置
* xCore SDK在使用前不需要通过Robot Assist打开相关功能。
* xCoreSDK为需要授权的功能，如遇“功能未授权”的错误信息，请联系客户支持人员
### 接口使用
见 *example*

# License

> Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. 
