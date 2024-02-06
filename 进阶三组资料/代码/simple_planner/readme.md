# simple_planner

## 文件结构

```
├── CMakeLists.txt
├── include
│   ├── cubic_spline
│   │   ├── cpprobotics_types.h
│   │   ├── cubic_spline.h
│   │   └── cubic_spline_ros.h
│   └── utility.h
├── launch
│   └── planner.launch
├── package.xml
├── readme.md
└── src
    ├── global_planner.cpp //用于导航
    ├── local_planner.cpp //用于导航
    └── pid_planner.cpp  //用于对位

```

## 功能说明
 实现无代价地图的定点导航

## 使用说明

1. 打开planner.launch 文件修改导航参数
2. 运行命令 `roslaunch simple_planner planner.launch `

## 其他说明
1. 订阅话题
- global_planner
 /clicked_point
 /initialpose

- local_planner 
 /global_planner/path
 
-pid_planner
 /item_pose
 
2. 发布话题
- global_planner
 /set_point
 /path
 
- local_planner 
 /cmd_vel
 /path

- pid_planner
 /calib_vel （需修改）

