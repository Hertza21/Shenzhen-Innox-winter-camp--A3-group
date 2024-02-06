#!/usr/bin/env python
# coding:utf-8
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from mavlink2ros.msg import RemoterInfo
from math import sin, cos, radians

rate = 1000

last_yaw = 0

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

yaw_ = 0

ref_yaw = 0

def remoter_info_callback(info: RemoterInfo):
    global last_yaw, ref_yaw
    twist_msg = Twist()
    # twist_msg.linear.x = info.channel_3 / rate
    # twist_msg.linear.y = -info.channel_2 / rate
    # twist_msg.angular.z = -info.channel_0 / 300
    # ref_velocity_x = chs.GetRemoter(3) * -10;
    # ref_velocity_y = chs.GetRemoter(2) * 10;
    # ref_velocity_roll = chs.GetRemoter(4) * 10;
    # v_x = cos((ref_yaw - chs.GetINS(1)) / 180.0 * 3.14159) * ref_velocity_y + sin((ref_yaw - chs.GetINS(1)) / 180.0 * 3.14159) * ref_velocity_x;
    # v_y = sin((ref_yaw - chs.GetINS(1)) / 180.0 * 3.14159) * ref_velocity_y - cos((ref_yaw - chs.GetINS(1)) / 180.0 * 3.14159) * ref_velocity_x;
    if info.switch_left == 0:
        reset_yaw()
        ref_yaw = get_yaw()
    elif info.switch_left == 1:
        ref_yaw = get_yaw()
    else:
        ref_yaw += info.channel_0 / 10000
    ref_vx = info.channel_2 / rate
    ref_vy = -info.channel_3 / rate
    ref_roll = info.wheel / rate
    vx = cos(radians(ref_yaw - get_yaw())) * ref_vx + sin(radians(ref_yaw - get_yaw())) * ref_vy
    vy = sin(radians(ref_yaw - get_yaw())) * ref_vx - cos(radians(ref_yaw - get_yaw())) * ref_vy
    twist_msg.linear.x = vy
    twist_msg.linear.y = vx
    # print(info.channel_0 / 300)
    twist_msg.angular.z = ref_roll
    pub.publish(twist_msg)

def get_yaw():
    print(yaw_ - last_yaw)
    return yaw_ - last_yaw

def reset_yaw():
    global last_yaw
    last_yaw = yaw_


def odom_info_callback(info: Odometry):
    global yaw_
    # yaw = info.pose.pose.orientation
    # roll, pitch, yaw = euler_from_quaternion(info.pose.pose.orientation)
    quat = info.pose.pose.orientation
    # 将四元数转换为欧拉角
    _, _, yaw_ = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

    # print(yaw_)

if __name__ == '__main__':
    # 创建节点
    rospy.init_node("test_remoter")
    rospy.Subscriber("remoter", RemoterInfo, remoter_info_callback)
    rospy.Subscriber("odom", Odometry, odom_info_callback)
    rospy.spin()
