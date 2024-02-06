# MAVLink2ROS

核心框架，实现底盘到 ROS 之间的通讯，使用 [MAVLink](https://mavlink.io/en/) 协议。

## 文件结构

```
.
|-- CMakeLists.txt
|-- README.md
|-- include
|   `-- FishCom             // FishCom 相关协议文件
|       |-- FishCom
|       |   `-- ...
|       |-- FishCom.xml     // <-- MAVLink 协议定义文件，可用与自动生成相关协议文件
|       |-- README.md       // <-- Here
|       `-- ...
|-- launch
|   `-- robot.launch
|-- msg                     // 自定义 ROS 消息定义文件
|   |-- RemoterInfo.msg     // 遥控器消息文件
|   |-- ServosCtrl.msg      // 舵机消息文件
|   `-- SingleServoCtrl.msg // 单个舵机消息文件
|-- package.xml
`-- src
    `-- robot.cpp           // 主代码
```

## 使用说明

- 通过 `roslaunch` 启动 `robot.launch` 文件，即可启动底盘通讯节点。
- 通过 `rostopic` 命令订阅或发布相关话题，实现底盘控制。

## 话题说明

1. 订阅话题
   - `/cmd_vel` 用于控制底盘 使用 [`geometry_msgs/Twist`](#geometry_msgs/Twist) 消息
   - `/servos_ctrl` 用于控制舵机 使用 [`ServosCtrl`](#ServosCtrl.msg) 消息
   - `/single_servo` 用于控制单个舵机 使用 [`SingleServoCtrl`](#SingleServoCtrl.msg) 消息
2. 发布话题
   - `/odom` 用于发布底盘里程计信息 使用 [`nav_msgs/Odometry`](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) 消息
   - `/remoter` 用于发布遥控器信息 使用 [`RemoterInfo`](#RemoterInfo.msg) 消息

## 消息说明

## geometry_msgs/Twist

ROS 文档：[geometry_msgs](http://wiki.ros.org/geometry_msgs) / [Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)

- `Vector3` `linear`
  - `x` 机器人 x 方向速度 (m/s)
  - `y` 机器人 y 方向速度 (m/s)
- `angular`
  - `z` 机器人旋转速度 (rad/s)

## RemoterInfo.msg

- `bool` `is_online`        遥控器是否在线
- `uint8` `switch_left`     左侧开关状态 0-2 下中上
- `uint8` `switch_right`    右侧开关状态 0-2 下中上
- `int16` `channel_0`       通道 0
- `int16` `channel_1`       通道 1
- `int16` `channel_2`       通道 2
- `int16` `channel_3`       通道 3
- `int16` `wheel`           滚轮状态

## ServosCtrl.msg

- `uint16` `servo1` 舵机 1 角度
- `uint16` `servo2` 舵机 2 角度
- `uint16` `servo3` 舵机 3 角度
- `uint16` `servo4` 舵机 4 角度
- `uint16` `servo5` 舵机 5 角度
- `uint16` `servo6` 舵机 6 角度
- `uint16` `servo7` 舵机 7 角度

## SingleServoCtrl.msg

- `uint8` `servo_id` 控制的舵机 ID
- `uint16` `target_value` 目标角度
- `duration` `duration` 控制时间 使用 std_msgs/Duration 消息
