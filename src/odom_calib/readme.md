# odom_calib

## 文件结构

```
├── CMakeLists.txt
├── include
│   └── odom_calib
├── launch
│   └── odom_calib.launch
├── package.xml
├── readme.md
└── src
    └── odom_calib_node.cpp

```

## 功能说明
  通过识别地图上的AprilTag ID大于100的tag，实现视觉定位修正编码器里程计定位
  发布矿车tag相对底盘位姿base2item_pose，实现对矿车tag的对位定位
## 使用说明

1. 开启map_server地图导入地图
2. 发布tag与map的静态坐标变换
3. 发布tag与base的坐标变换
3. 开启摄像头识别apriltag节点
4. 运行命令 `roslaunch odom_calib odom_calib.launch `

## 其他说明

需要的数据

- tf变换数据需包含
  /ep_camera_frame
  /base_link
  /odom
  /tag_xx
  
- 订阅话题
 /tag_detections
 -发布话题
  /item_pose



