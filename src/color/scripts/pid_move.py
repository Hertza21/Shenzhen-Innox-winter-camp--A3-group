import rospy
from std_msgs.msg import Int16MultiArray
import time
from geometry_msgs.msg import Twist
# 创建一个全局变量来保存接收到的消息
received_message = None

def chatter_callback(data):
    """
    回调函数，当接收到chatter话题的消息时被调用
    :param data: 从chatter话题接收到的数据，这里是std_msgs/Int16MultiArray类型
    """
    global received_message
    received_message = data.data.copy()  # 将接收到的消息保存到全局变量中


class PIDController:
    def __init__(self, kp=0.005, ki=0.0, kd=0.0, output_limits=(-1, 1), integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limit = integral_limit or float('inf')
        self.setpoint = 450
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

    def update(self, current_value):
        global received_message
        error = self.setpoint - received_message

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 比例项
        p_term = self.kp * error

        # 积分项
        self.integral += error * dt
        if abs(self.integral) > self.integral_limit:
            self.integral = self.integral_limit if self.integral > 0 else -self.integral_limit
        i_term = self.ki * self.integral

        # 微分项
        d_term = 0 if dt == 0 else self.kd * (error - self.previous_error) / dt
        self.previous_error = error

        # 计算PID输出并限制在指定范围内
        output = p_term + i_term + d_term
        output = max(min(output, self.output_limits[1]), self.output_limits[0])

        return output


# 假设你已经初始化了节点
rospy.init_node('I_am_homo')

def publish_linear_speed(speed):
    # 创建Twist消息对象
    twist_msg = Twist()

    # 设置x方向的速度值
    twist_msg.linear.x = speed

    # 其他方向的速度和角速度设置为0
    twist_msg.linear.y = 0.0
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = 0.0

    # 发布cmd_vel话题
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub.publish(twist_msg)


# 初始化ROS节点
rospy.init_node('chatter_subscriber', anonymous=True)

#创建Subscriber，订阅名为chatter的话题，并指定回调函数为chatter_callback
subscriber = rospy.Subscriber('/chatter', Int16MultiArray, chatter_callback)
# 初始化PID控制器实例
pid = PIDController(kp=0.002, ki=0, kd=0.002)
# 运行其他代码或循环以处理received_message变量中的消息
while not rospy.is_shutdown():
    if received_message is not None:
        while True:
            # 更新received_message的值（这里假设从某个传感器读取）
            subscriber = rospy.Subscriber('/chatter', Int16MultiArray, chatter_callback)

            # 调整电机速度
            speed = pid.update(received_message)
            publish_linear_speed(speed)  # 请替换为实际设置速度的函数或操作

            # 等待下一个采样周期
            time.sleep(0.01)  # 根据实际情况调整采样周期

    # 维持节点运行并检查新的消息
    rospy.sleep(0.1)


