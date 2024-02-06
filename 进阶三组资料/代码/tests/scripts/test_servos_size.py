#!/usr/bin/env python
# coding:utf-8
import rospy
from mavlink2ros.msg import ServosCtrl

turn_delay = 2

def set_all_servos_value(value):
    servos_list = ServosCtrl()
    servos_list.servo1 = value
    servos_list.servo2 = value
    servos_list.servo3 = value
    servos_list.servo4 = value
    servos_list.servo5 = value
    servos_list.servo6 = value
    servos_list.servo7 = value
    pub.publish(servos_list)

if __name__ == '__main__':
    # 创建节点
    rospy.init_node("test_servos")
    pub = rospy.Publisher('servos_ctrl', ServosCtrl, queue_size=10)
    begin = 460
    end = 2400
    step = 10
    while not rospy.is_shutdown():
        # 往复运动检测电机支持的最大转速
        set_all_servos_value(begin)
        rospy.sleep(turn_delay)
        set_all_servos_value(end)
        select = input("Q: set begin higher\nA: set begin lower\nW: set end higher\nS: set end lower\nE: set step higher\nD: set step lower\n"
                       "E: set step higher\nD: set step lower\n").upper()
        if select == 'Q':
            begin += step
        elif select == 'A':
            begin -= step
        elif select == 'W':
            end += step
        elif select == 'S':
            end -= step
        elif select == 'E':
            step += 1
        elif select == 'D':
            step -= 1
        # else:
        #     print("invalid input")
        print("begin: {}, end: {}, step: {}".format(begin, end, step))



