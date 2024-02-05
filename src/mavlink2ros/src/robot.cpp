#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <thread>
#include <iostream>
#include "FishCom/mavlink.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>

#include "mavlink2ros/RemoterInfo.h"
#include "mavlink2ros/ServosCtrl.h"
#include "mavlink2ros/SingleServoCtrl.h"
#include "mavlink2ros/ManageCtrl.h"

// 定义舵机复位角度和时间
#define SERVO_RESET_ANGLE 500
#define SERVO_RESET_TIME 0

// 定义默认的机器人控制参数
#define DEFAULT_ENABLE_CHASSIS 1
#define DEFAULT_ENABLE_SERVOS 0
#define DEFAULT_RESET_QUATERNION 1

// Servos Ctrl Task
struct ServoCtrlTask
{
    uint8_t servo_id;
    uint16_t servo_value;
    int16_t change_value;
    uint16_t origin_value;
    uint16_t target_value;
    ros::Time start_time;
    ros::Time end_time;
    ServoCtrlTask(uint8_t id) : servo_id(id), servo_value(0), change_value(0), origin_value(0),
                                target_value(SERVO_RESET_ANGLE), start_time(ros::Time::now()),
                                end_time(ros::Time::now()+ros::Duration(SERVO_RESET_TIME)) {}
};



class RobotController {
public:
    RobotController(const std::string& port, int baud_rate) : 
        serial_device(port, baud_rate, serial::Timeout::simpleTimeout(1000)),
        chassis_odom_pub_(nh.advertise<nav_msgs::Odometry>("odom", 50)),
        remoter_pub_(nh.advertise<mavlink2ros::RemoterInfo>("remoter", 50))
    {
        if (!serial_device.isOpen()) {
            std::cerr << "Failed to open serial port." << std::endl;
        }

        motor_sub = nh.subscribe("cmd_vel", 1000, &RobotController::cmdVelCallback, this);
        servos_sub = nh.subscribe("servos_ctrl", 100, &RobotController::servosCallback, this);
        single_servo_sub = nh.subscribe("single_servo", 100, &RobotController::singleServoCallback, this);
        manage_sub = nh.subscribe("manage_ctrl", 100, &RobotController::manageCallback, this);
        receive_thread = std::thread(&RobotController::receiveData, this);

        // Regularly Msg
        manage_timer = nh.createTimer(ros::Duration(1.0), &RobotController::sendManageInfoRegularly, this);
        motor_control_timer = nh.createTimer(ros::Duration(0.01), &RobotController::sendMotorCtrlInfoRegularly, this);
        servos_control_timer = nh.createTimer(ros::Duration(0.01), &RobotController::sendServosCtrlInfoRegularly, this);
        
        chassis_odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom",1);
        remoter_pub_ = nh.advertise<mavlink2ros::RemoterInfo>("remoter", 1);
        odom_.header.frame_id = "odom";
        odom_.child_frame_id = "base_link";
        odom_tf_.header.frame_id = "odom";
        odom_tf_.child_frame_id = "base_link";
    }

    ~RobotController() {
        if (receive_thread.joinable()) {
            receive_thread.join();
        }
    }

    void sendManageInfoRegularly(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(manage_mutex); // 确保线程安全
        sendManageInfo(enable_chassis, enable_servos, reset_quaternion);
    }

    void sendMotorCtrlInfoRegularly(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(twist_mutex); // 确保线程安全
        sendCtrlInfo(current_twist.linear.x, current_twist.linear.y, current_twist.angular.z);
    }

    void sendServosCtrlInfoRegularly(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(servos_twist_mutex); // 确保线程安全
        // 循环遍历所有舵机，检查是否需要更新舵机值
        for(int i = 0; i < 7; i++){
            ServoCtrlTask& servo = servos_list[i];
            if (servo.servo_value != servo.target_value) {
                // 如果舵机值需要更新
                if (ros::Time::now() > servo.end_time) {
                    // 时间已经结束，强制跟新舵机值
                    servo.servo_value = servo.target_value;
                } else {
                    auto interval_time = servo.end_time - servo.start_time;
                    auto rate = (ros::Time::now() - servo.start_time).toSec() / interval_time.toSec();
                    servo.servo_value = servo.origin_value + servo.change_value * rate;
                }
            } else {
                // 如果舵机值不需要更新
                servo.servo_value = servo.target_value;
            }
        }
        const uint16_t servos_value_list[7] = {servos_list[0].servo_value, servos_list[1].servo_value, servos_list[2].servo_value,
                                               servos_list[3].servo_value, servos_list[4].servo_value, servos_list[5].servo_value,
                                               servos_list[6].servo_value};
        std::cout << servos_value_list[0] << std::endl;
        sendServosCtrlInfo(servos_value_list);
    }

    void sendManageInfo(bool enable_chassis,bool enable_servos,bool reset_quaternion){
        mavlink_message_t *msg = (mavlink_message_t *)malloc(sizeof(mavlink_message_t));
        memset(msg, 0, sizeof(mavlink_message_t));

        uint16_t Txlen = 0;
        uint8_t *txbuf = (uint8_t *)malloc(MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN + 12);

        mavlink_msg_chs_manage_info_pack(
            CHS_SYSTEM_ID::CHS_ID_ORANGE,
            CHS_SYSTEM_ID::CHS_ID_ORANGE, 
            msg, enable_chassis, enable_servos, reset_quaternion);
        
        Txlen = mavlink_msg_to_send_buffer(txbuf, msg);

        serial_device.write(txbuf, Txlen);

        free(msg);
        free(txbuf);
    }

    void sendCtrlInfo(float vx, float vy, float vz){
        mavlink_message_t *msg = (mavlink_message_t *)malloc(sizeof(mavlink_message_t));
        memset(msg, 0, sizeof(mavlink_message_t));

        uint16_t Txlen = 0;
        uint8_t *txbuf = (uint8_t *)malloc(MAVLINK_MSG_ID_CHS_CTRL_INFO_LEN + 12);

        mavlink_msg_chs_ctrl_info_pack(
            CHS_SYSTEM_ID::CHS_ID_ORANGE,
            CHS_SYSTEM_ID::CHS_ID_ORANGE, 
            msg, 
            -vy, vx, vz);
        
        Txlen = mavlink_msg_to_send_buffer(txbuf, msg);

        serial_device.write(txbuf, Txlen);

        free(msg);
        free(txbuf);
    }

    void sendServosCtrlInfo(const uint16_t* servos){
        mavlink_message_t *msg = (mavlink_message_t *)malloc(sizeof(mavlink_message_t));
        memset(msg, 0, sizeof(mavlink_message_t));

        uint16_t Txlen = 0;
        uint8_t *txbuf = (uint8_t *)malloc(MAVLINK_MSG_ID_CHS_SERVOS_INFO_LEN + 12);

        mavlink_msg_chs_servos_info_pack(
            CHS_SYSTEM_ID::CHS_ID_ORANGE,
            CHS_SYSTEM_ID::CHS_ID_ORANGE, 
            msg, 
            servos);
        
        Txlen = mavlink_msg_to_send_buffer(txbuf, msg);

        serial_device.write(txbuf, Txlen);

        free(msg);
        free(txbuf);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        current_twist = *msg;
    }

    void servosCallback(const mavlink2ros::ServosCtrl servos_msg) {
        // servos from 499 to 2499
        servos_list[0].target_value = servos_msg.servo1;
        servos_list[1].target_value = servos_msg.servo2;
        servos_list[2].target_value = servos_msg.servo3;
        servos_list[3].target_value = servos_msg.servo4;
        servos_list[4].target_value = servos_msg.servo5;
        servos_list[5].target_value = servos_msg.servo6;
        servos_list[6].target_value = servos_msg.servo7;
    }

    void singleServoCallback(const mavlink2ros::SingleServoCtrl servo_msg) {
        // servo_id from 0 to 6, servo_speed from 499 to 2499
        if (servo_msg.servo_id < 0 || servo_msg.servo_id > 6) {
            ROS_WARN("Servo ID is out of range.");
            return;
        }
        ServoCtrlTask& servo = servos_list[servo_msg.servo_id];
        servo.start_time = ros::Time::now();
        servo.end_time = ros::Time::now() + servo_msg.duration;
        servo.origin_value = servo.servo_value;
        servo.target_value = servo_msg.target_value;
        servo.change_value = servo_msg.target_value - servo.servo_value;
    }

    void manageCallback(const mavlink2ros::ManageCtrl manage_msg) {
        enable_chassis = manage_msg.enable_chassis;
        enable_servos = manage_msg.enable_servos;
        reset_quaternion = manage_msg.reset_quaternion;
    }

    void receiveData() {
        while (ros::ok()) {
            if (serial_device.available()) {
                mavlink_message_t *msg = (mavlink_message_t *)malloc(sizeof(mavlink_message_t));
                memset(msg, 0, sizeof(mavlink_message_t));

                mavlink_status_t status;
                uint8_t rxbuf[512];
                size_t RxLen = serial_device.read(rxbuf, sizeof(rxbuf));

                mavlink_chs_odom_info_t odom_info;
                mavlink_chs_ctrl_info_t ctrl_info;
                mavlink_chs_motor_info_t motor_info;
                mavlink_chs_servos_info_t servos_info;
                mavlink_chs_manage_info_t manage_info;
                mavlink_chs_remoter_info_t remoter_info;

                for (size_t i = 0; i < RxLen; ++i) {
                    // printf("Byte %ld: 0x%02X\n", i, rxbuf[i]);
                    if (mavlink_parse_char(MAVLINK_COMM_0, rxbuf[i], msg, &status)) {
                        // printf("第 %ld 个包，解包成功\n",i);
                        // printf("%d",msg->msgid);
                        switch (msg->msgid) {
                            case MAVLINK_MSG_ID_CHS_ODOM_INFO: {
                                mavlink_msg_chs_odom_info_decode(msg, &odom_info);
                                // printf("received odom info! \n");
                                // printf("vx: %f,vy: %f,vz: %f \n",odom_info.vx,odom_info.vy,odom_info.vw);
                                // printf("quaternion: ");
                                // for(int i = 0; i < 4; i++) printf("%f ", odom_info.quaternion[i]);

                                std::swap(odom_info.vx,odom_info.vy);
                                odom_info.vy = -odom_info.vy;
                                
                                // printf("\n");

                                current_time = ros::Time::now();
                                double dt = (current_time - last_time).toSec();

                                // 使用从 MAVLink 消息接收到的速度来更新位置和姿态
                                double delta_x = (odom_info.vx * cos(th) - odom_info.vy * sin(th)) * dt;
                                double delta_y = (odom_info.vx * sin(th) + odom_info.vy * cos(th)) * dt;
                                double delta_th = odom_info.vw * dt;

                                x += delta_x;
                                y += delta_y;
                                th += delta_th;

                                // 更新里程计消息
                                odom_.header.stamp = current_time;
                                odom_.twist.twist.linear.x = odom_info.vx;
                                odom_.twist.twist.linear.y = odom_info.vy;
                                odom_.twist.twist.angular.z = odom_info.vw;

                                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
                                odom_.pose.pose.position.x = x;
                                odom_.pose.pose.position.y = y;
                                odom_.pose.pose.position.z = 0.0;
                                odom_.pose.pose.orientation = odom_quat;

                                // 发布里程计消息
                                chassis_odom_pub_.publish(odom_);

                                // 发布 TF 变换
                                odom_tf_.header.stamp = current_time;
                                odom_tf_.transform.translation.x = x;
                                odom_tf_.transform.translation.y = y;
                                odom_tf_.transform.translation.z = 0.0;
                                odom_tf_.transform.rotation = odom_quat;
                                tf_broadcaster_.sendTransform(odom_tf_);

                                last_time = current_time;
                                break;
                            }
                            case MAVLINK_MSG_ID_CHS_REMOTER_INFO: {
                                mavlink_msg_chs_remoter_info_decode(msg, &remoter_info);
                                remoter_info_.is_online = remoter_info.switch_messgae & 0x01;
                                remoter_info_.switch_left = (remoter_info.switch_messgae & 0x06) >> 1;
                                remoter_info_.switch_right = (remoter_info.switch_messgae & 0x18) >> 3;
                                remoter_info_.channel_0 = remoter_info.channel_0;
                                remoter_info_.channel_1 = remoter_info.channel_1;
                                remoter_info_.channel_2 = remoter_info.channel_2;
                                remoter_info_.channel_3 = remoter_info.channel_3;
                                remoter_info_.wheel = remoter_info.wheel;
                                remoter_pub_.publish(remoter_info_);
                                break;
                            }
                        }
                    }
                }
                free(msg);
            }
            usleep(1000);
        }
    }

private:
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber motor_sub;
    ros::Subscriber servos_sub;
    ros::Subscriber single_servo_sub;
    ros::Subscriber manage_sub;
    serial::Serial serial_device;
    std::thread receive_thread;
    geometry_msgs::Twist current_twist; 

    // Timer and Mutex
    ros::Timer manage_timer;
    ros::Timer motor_control_timer;
    ros::Timer servos_control_timer;
    std::mutex twist_mutex;
    std::mutex servos_twist_mutex;
    std::mutex manage_mutex;

    // Publisher
    tf::TransformBroadcaster tf_broadcaster_;//! ros chassis odometry tf broadcaster
    ros::Publisher chassis_odom_pub_;//! ros odometry message publisher
    ros::Publisher remoter_pub_;    // Remoter publisher
    geometry_msgs::TransformStamped odom_tf_;//! ros chassis odometry tf
    nav_msgs::Odometry odom_;//! ros odometry message
    mavlink2ros::RemoterInfo remoter_info_;
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    ros::Time current_time = ros::Time::now(), last_time = ros::Time::now();

    // Servos Ctrl
    ServoCtrlTask servos_list[7] = {ServoCtrlTask(0), ServoCtrlTask(1), ServoCtrlTask(2),
                                    ServoCtrlTask(3), ServoCtrlTask(4), ServoCtrlTask(5),
                                    ServoCtrlTask(6)};

    // Manage Settings
    bool enable_chassis = DEFAULT_ENABLE_CHASSIS;
    bool enable_servos = DEFAULT_ENABLE_SERVOS;
    bool reset_quaternion = DEFAULT_RESET_QUATERNION;
}; 

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller");
    RobotController controller("/dev/robomaster", 115200);
    ros::spin();
    return 0;
}
