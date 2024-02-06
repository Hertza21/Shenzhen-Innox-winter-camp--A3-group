#!/usr/bin/env python
# coding:utf-8
import rospy
from mavlink2ros.msg import ServosCtrl


if __name__ == '__main__':
    # 创建节点
    rospy.init_node("test_servos")
    pub = rospy.Publisher('servos_ctrl', ServosCtrl, queue_size=10)
    r = rospy.Rate(5) # 2
    while not rospy.is_shutdown():
        for i in range(500, 2500, 20):
            if rospy.is_shutdown():
                break
            print(i)
            servos_list = ServosCtrl()
            servos_list.servo1 = i
            servos_list.servo2 = i
            servos_list.servo3 = i
            servos_list.servo4 = i
            servos_list.servo5 = i
            servos_list.servo6 = i
            servos_list.servo7 = i
            pub.publish(servos_list)
            r.sleep()
        for i in range(2500, 500, -20):
            if rospy.is_shutdown():
                break
            print(i)
            servos_list = ServosCtrl()
            servos_list.servo1 = i
            servos_list.servo2 = i
            servos_list.servo3 = i
            servos_list.servo4 = i
            servos_list.servo5 = i
            servos_list.servo6 = i
            servos_list.servo7 = i
            pub.publish(servos_list)
            r.sleep()
