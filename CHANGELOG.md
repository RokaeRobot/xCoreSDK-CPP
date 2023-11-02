# CHANGELOG

## v0.3.4 2023-11-02
* 兼容性  
  * xCore >= v2.1.0.15 (三位发布号v2.1.0)
  * xMateModel模型库增加支持Windows-64 Debug版本
* 新增
  * Robot类的默认构造，带IP地址参数的连接机器人接口connectToRobot(remoteIP, localIP)
  * 读取设置软限位接口getSoftLimit(), setSoftLimit()
  * 协作机器人读取末端力矩接口getEndTorque()
  * 碰撞检测触发行为增加柔顺停止(StopLevel::suppleStop)和柔顺度选项
  * xMateCR和xMateSR机型奇异规避相关接口: 奇异规避&平行基座Jog，设置奇异规避模式运动setAvoidSingularity(), 
  * 非实时运动信息反馈增加点位距离过近的报警信息(EventInfoKey::MoveExecution::Remark)
  * 工具/工件/基坐标系标定接口calibrateFrame()
  * 螺旋线运动指令MoveSPCommand
  * 设置是否严格遵循轴配置数据接口setDefaultConfOpt()
  * 模型库支持所有已知协作机型，增加支持XMS3, XMS4, XMC18, XMC20
* 修复&优化
  * 在工具工件坐标系下Jog机器人，由原来的使用RobotAssist右上角选择的工具工件，改为使用通过setToolset()设置的工具工件坐标系
  * 全圆指令MoveCFCommand参数全圆执行角度(angle)单位由度数改为弧度
  * 删除ForceControlFrameType枚举类，setFcCoor()中坐标系类型参数改为FrameType
  * 修复实时模式运动中发生异常（如急停）不能恢复、只能重新运行的问题
  * 移除模型库对glog的依赖
  * 修复计算逆解接口不回复，或者用时较长的问题
  * 修复因网络异常没有处理造成的实时收发线程可能崩溃的问题
  * 其它已知问题

## v0.3.3 2023-08-23
* 兼容性
  * 增加Linux下不依赖模型库的xCoreSDK.so, 可用于编译动态库
* 修复&优化
  * 通过moveAppend()发送运动指令可能不执行的问题
  * 实时模式轴空间阻抗控制，放宽阻抗系数上限到3000, 300
  * RL工程相关接口没有检查模式的问题

## v0.3.2 2023-07-04
* 兼容性
  * xCore >= v2.0.1
  * xMateModel模型库支持Linux x86_64; 及Windows Release编译类型
* 新增
  * 运动指令MoveCF
  * 设置和打开关闭碰撞检测接口; 碰撞监测功能
* 修复
  * 实时模式急停后恢复运动的问题; 及其它已知问题

## v0.3.1 2023-05-03
* 兼容性
  * xCore >= v2.0.0.7
  * 增加动态库；xMateModel及相关接口除外，仅支持Linux静态库
* 新增&优化
  * 非实时运动指令增加暂停、继续、获取运动指令执行信息功能，新增接口moveStart(), moveAppend(); 及设置事件回调setEventWatcher();
  * 笛卡尔目标点增加偏移选项(Offs/Reltool)
  * 增加全局调整运动速率接口adjustSpeedOnline()

## v0.3.0 2023-03-07
* 兼容性
  * xCore >= v2.0.0.1
* 新增&优化
  * xCore版本匹配检查
  * 工业六轴机型支持实时模式位置控制
  * 支持不开启实时模式的情况下读取状态数据
  * 更新Eigen库
  * 增加设置IO仿真模式和设置DI
  * ”XMate“开头的类改成为”xMate"开头
  * projectPointToMain()改为ppToMain()
* 修复
  * loadProject()加载工程增加工程是否存在的检查

## v0.2.8 2023-02-15
* 新增&优化
  * 合并CartesianPosition & CartesianPose
  * 删除append()&executeCommands()
  * 增加FollowPosition目标跟随
  * xMateModel增加适配CR&SR机型

## v0.2.7 2023-02-01
* 新增&优化
  * 实时模式状态数据改为同步接收；读取接口统一为getStateData(); 增加updateRobotState()更新状态数据
  * 去掉接收数据以及周期调度的间隔参数，统一为间隔为1ms

## v0.2.6 2023-01-12
* 修复
  * 调用append()发送多条运动指令后自动开始执行的问题，修改回发送后立即开始执行。append()和executeCommands()接口弃用。
  * 没有处理网络断开、解析回复失败等发生错误时抛出的异常
* 新增&优化
  * executeCommand(), 接口功能与v0.2.0之前版本的append()一样
  * 删除data_types.h中数据结构getter&setter
  * CartesianPosition类增加臂角值"elbow"
  * 删除xMateModel库getJointPosWithConf()接口
  * Debug版本加一点错误打印
  
## v0.2.5 2023-01-09
* 兼容性
  * 增加MSVC Debug版本库
* 新增
  * 实时模式上位机规划MoveC指令
  
## v0.2.4 2023-01-05
* 兼容性
  * xCore版本 >= v1.7.0.9 (影响与原版RCI客户端切换使用问题)
* 修复
  * 实时模式运动异常无法回传到主线程的问题
  * 一些接口的数据设置返回结果没有赋给错误码
* 新增
  * 查询控制器日志
  * 切换回原版RCI客户端的接口
  * 数据转换工具类

## v0.2.3 2022-12-27
* 兼容性
  * xCore版本 >= v1.7.0.7 (仅影响运动RL工程的速率设置)
* 新增
  * 加载、运行、暂停运行RL工程相关函数
  * jog机器人
  * 读写模拟量信号，读写寄存器

## v0.2.2 2022-12-19
* 兼容性
  * xCore版本 >= v1.7.0.6 (仅影响RCI模式下打开关闭拖动)
* 新增
  * 实时MoveL & MoveJ运动指令
  * 连接机器人时检查工业/协作类型
* 修复
  * RCI模式下打开/关闭拖动没有调用RCI对应方法的问题

## v0.2.1 2022-12-09
* 新增
  * 断开/重新连接到RCI服务器接口
  * 设置丢包阈值接口，微调整控制器内计算丢包率方法
  * 等待接收控制器发送的实时状态信息超时时间为3秒

## v0.2.0 2022-12-07
* 兼容性
  * xCore版本 >= v1.7.0.5
* 新增
  * RCI实时控制，关节/笛卡尔位置/阻抗控制
  * xMateModel模型库，只支持Linux系统
  * 非实时运动指令增加executeCommands(),将发送运动指令到缓存和开始运动分离，调用append()后机器人不再直接开始运动
* 修复
  * append()单条指令转弯区不生效的问题(和第3条新增描述的内容相关)
  * 优化append()多条指令默认只用第一条指令的转弯区和速度设置的问题。每个单独设置了转弯区或速度的指令都生效。

## v0.1.5 2022-11-07
* 修复
  * 缺少头文件导致的编译问题

## v0.1.4 2022-10-24
* 修复
  * 由于轴配置数据默认为0而导致的计算逆解错误问题。同时修改正逆解函数参数和返回值，带上轴配置数据

## v0.1.3 2022-10-19
* 兼容性
  * 编译器
    * MSVC 14.1+ (Visual Studio 2017 version 15.0)
* 修复
  * dll导入导出属性导致的静态库链接问题
  * 断开机器人连接或Robot对象被析构时，仅当机器人在执行SDK发送的运动指令时才停止运动

## v0.1.2 2022-10-17
* 新增
  * 下发运动指令接口append()增加vector类型作为参数
  
## v0.1.1 2022-10-11
* 兼容性
  * xCore版本 >= V1.6.2
* 新增
  * 路径录制与回放相关接口，及示例程序
* 修复
  * 直接调用关闭拖动接口会报错
  * 机器人处于摩擦力/动力学辨识等状态时，查询状态返回信息不准确

## v0.1.0 2022-09-23
* 兼容性
  * xCore版本 >= V1.6.1
  * 开发语言: C++
  * 操作系统及编译器
    * Windows10 - MSVC 14.2+ (Visual Studio 2019 version 16.0)
    * Ubuntu 18.04+
* 新增
  * 机器人基本操作, 包括上下电，切换模式，查询状态位姿，DI/DO设置，打开关闭拖动等
  * 非实时运动控制，使用控制器内部路径规划完成MoveJ/MoveL/MoveAbsJ/MoveC运动指令