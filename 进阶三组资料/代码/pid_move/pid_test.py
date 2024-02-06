import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


class PIDController2D:
    def __init__(self, kp_x=0.3, ki_x=0.0, kd_x=0.0,
                 kp_y=0.3, ki_y=0.0, kd_y=0.0, max_output=1.0):
        self.kp_x = kp_x
        self.ki_x = ki_x
        self.kd_x = kd_x
        self.integral_x = 0.0
        self.previous_error_x = 0.0

        self.kp_y = kp_y
        self.ki_y = ki_y
        self.kd_y = kd_y
        self.integral_y = 0.0
        self.previous_error_y = 0.0

        self.max_output = max_output

        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def update(self, target_position, current_position):
        
        global error_x
        error_x = target_position[0] - current_position[0]
        derivative_x = error_x - self.previous_error_x if self.previous_error_x else 0.0
        self.integral_x += error_x
        output_x = min(max(self.kp_x * error_x + self.ki_x * self.integral_x + self.kd_x * derivative_x, -self.max_output), self.max_output)
        
        global error_y
        error_y = target_position[1] - current_position[1]
        derivative_y = error_y - self.previous_error_y if self.previous_error_y else 0.0
        self.integral_y += error_y
        output_y = min(max(self.kp_y * error_y + self.ki_y * self.integral_y + self.kd_y * derivative_y, -self.max_output), self.max_output)

        # 更新前一次误差值
        self.previous_error_x = error_x
        self.previous_error_y = error_y

        # 发布Twist消息
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = output_x
        cmd_vel_msg.linear.y = output_y
        self.twist_pub.publish(cmd_vel_msg)

# 初始化ROS节点
rospy.init_node('pid_controller')

# 创建PID控制器实例，设置合适的PID参数
pid_controller = PIDController2D(kp_x=1.0, ki_x=0.5, kd_x=0.1,
                                 kp_y=1.0, ki_y=0.5, kd_y=0.1, max_output=0.5)

# 假设current_robot_position是从odom消息中提取出来的
def odom_callback(odom_msg):
    current_position = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)
    x = float(input('请输入x坐标：'))
    y = float(input('请输入y坐标：'))
    # 设置目标位置
    target_position = (x,y)  # 示例目标位置，实际应用中应根据需要动态设置

    pid_controller.update(target_position, current_position)
    while error_x >= 0.1 or error_y >= 0.1:
        pid_controller.update(target_position, current_position)

# 订阅odom消息
odom_subscriber = rospy.Subscriber('/odom', Odometry, odom_callback)

# 保持节点运行
rospy.spin()




