#PID_move程序技术文档 2024.2.2
##程序概述
-本程序运用Python实现沙包对位的PID控制 \
##所需环境
-Python3.7或以上 \
##程序函数分析
-chatter_callback()：Subscriber的回调函数。 \
-class PIDController：功能：包含两个函数，实现底盘运动速度的计算。\
-                  输入：底盘的x坐标，P,I,D参数。\
-                  输出：底盘的x方向的速度。\
-publish_linear_speed()：功能：发布cmd_vel消息，控制底盘运动 \
##未来展望
-等待上机实测ing \
