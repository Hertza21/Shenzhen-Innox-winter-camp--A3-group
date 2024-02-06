# from servos_ctrl.utils import Servo, delay, add_task
# import os
# print(os.getcwd())

"""utils.py"""
#!/usr/bin/env python
# coding:utf-8
import rospy
from std_msgs.msg import String
from mavlink2ros.msg import ServosCtrl, SingleServoCtrl


rospy.init_node('servos_ctrl')
_servos_ctrl_pub = rospy.Publisher('servos_ctrl', ServosCtrl, queue_size=10)
_single_servo_ctrl_pub = rospy.Publisher('single_servo', SingleServoCtrl, queue_size=10)

task_list = {}


def set_servos_ctrl(servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8):
    msg = ServosCtrl()
    msg.servo1 = servo1
    msg.servo2 = servo2
    msg.servo3 = servo3
    msg.servo4 = servo4
    msg.servo5 = servo5
    msg.servo6 = servo6
    msg.servo7 = servo7
    msg.servo8 = servo8
    _servos_ctrl_pub.publish(msg)


def set_single_servo_ctrl(servo_id: int, target_value: int, duration: float):
    msg = SingleServoCtrl()
    msg.servo_id = servo_id
    msg.target_value = target_value
    msg.duration = rospy.Duration(max(duration, 0))  # 最小值为 0
    msg.duration
    _single_servo_ctrl_pub.publish(msg)


def delay(duration: float):
    rospy.sleep(rospy.Duration(duration))


class Servo:
    def __init__(self, servo_id: int, init_value: int, min_value: int, max_value: int):
        """创建一个舵机便捷控制对象

        Args:
            servo_id (int): 舵机 ID [0-6]
            init_value (int): 舵机初始值
            min_value (int): 舵机最小值
            max_value (int): 舵机最大值
        """
        self.servo_id = servo_id
        self.init_value = init_value
        self.min_value = min_value
        self.max_value = max_value
        self.current_value = init_value
        self.set_value(init_value, 0)

    def set_value(self, value: int, duration: float, wait: bool = False):
        value = max(self.min_value, min(self.max_value, value))
        set_single_servo_ctrl(self.servo_id, value, duration)
        self.current_value = value
        if wait:
            delay(duration)

    def set_value_directly(self, value: int):
        value = max(self.min_value, min(self.max_value, value))
        set_single_servo_ctrl(self.servo_id, value, 0)
        self.current_value = value

    def __add__(self, value: int):
        self.set_value(self.current_value + value, 0)

    def __sub__(self, value: int):
        self.set_value(self.current_value - value, 0)

    def copy(self, servo_id: int = 0):
        """便捷复制舵机对象

        Args:
            servo_id (int, optional): 新舵机的 ID. Defaults to 0.

        Returns:
            Servo: 新舵机对象
        """
        return Servo(servo_id, self.init_value, self.min_value, self.max_value)


def add_task(task_name: str):
    global task_list
    print("add task", task_name)
    def out_wrapper(func):
        task_list[task_name] = func
        def warp(*args, **kwargs):
            return func(*args, **kwargs)
        return warp
    print(task_list)
    return out_wrapper


def task_callback(msg: String):
    task_name = msg.data
    print("Call Task: ", msg.data)
    if task_name in task_list:
        print(task_list[task_name])
        task_list[task_name]()
    else:
        rospy.logwarn(f'CAN NOT find servos task: {task_name}')

_task_sub = rospy.Subscriber('servos_task', String, task_callback)

r"""
\\        //
 \\______//   <== claw
 |        |   <== claw_servo
 ----------
     ||
 ----------
 |        |   <== joint_servo1
 ----------
     ||
 ----------
 |        |   <== joint_servo2
 ----------
     ||
 ----------
 |        |   <== joint_servo3
 ----------
  ||
|----|
|    |
|    |      <== bottom_servo
|    |
|----|
"""
# 定义机械爪舵机
claw_servo = Servo(0, 1160, 800, 1200)   # 爪子舵机     1160：Close     800: Open

# 调小：向前    调大：向后
joint_servo1 = Servo(1, 1400, 400, 2400)  # 关节舵机 1
joint_servo2 = Servo(2, 1400, 400, 2400)  # 关节舵机 2
joint_servo3 = Servo(3, 1400, 400, 2400)  # 关节舵机 3

bottom_servo = Servo(4, 1400, 700, 2100)  # 底座舵机 调小：向后 调大：向前


@add_task('open_claw')
def open_claw():
    claw_servo.set_value(800, 1.5)
    delay(1.5)


@add_task('close_claw')
def close_claw():
    claw_servo.set_value(1200, 1.5)
    delay(1.5)


@add_task('expand')
def expand():
    bottom_servo.set_value(1600, 1)
    delay(0.5)
    joint_servo3.set_value(1400, 1)
    delay(0.5)
    joint_servo2.set_value(1400, 1)
    delay(0.5)
    joint_servo1.set_value(1400, 1)
    delay(0.5)
    bottom_servo.set_value(1900, 1)


@add_task('shrink')
def shrink():
    joint_servo1.set_value(2300, 0.8)
    delay(0.5)
    joint_servo2.set_value(1700, 1)
    delay(0.5)
    joint_servo3.set_value(2500, 1)
    delay(0.5)
    bottom_servo.set_value(1400, 1)
    delay(0.5)

rospy.spin()
