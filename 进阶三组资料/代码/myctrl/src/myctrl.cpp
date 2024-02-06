
#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Int16MultiArray.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#define speeded 0.02
#define uppered 700
#define lowered 600

class talker_listener
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

public:
    talker_listener()
    {
        sub = nh.subscribe("chatter", 5, &talker_listener::Callback, this);
        pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }
    void Callback(const std_msgs::Int16MultiArray::ConstPtr &msg);
};

void talker_listener::Callback(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
    geometry_msgs::Twist cmd_vel_;
    short bok = 0, bok_ = 0;
    for (int i = 0; i <= 4; i++)
    {
        std::cout << msg->data[i] << ' ';
        bok = bok + (msg->data[i] == -1 ? 1 : 0);
        bok_ = (msg->data[i] != -1 ? i : bok_);
    }
    std::cout << std::endl;
    // if (bok == 4)
    // {
        
        short x = (msg->data[1] ==-1?(lowered+uppered)/2:msg->data[1]);
        x = x > uppered ? uppered : (x < lowered ? lowered : x);
        switch (x)
        {
        case lowered:
            cmd_vel_.linear.y = speeded;
            break;
        case uppered:
            cmd_vel_.linear.y = -1.0*speeded;
            break;
        default:
            cmd_vel_.linear.y=0.0;
            break;
        }
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.linear.z = 0.0;
        cmd_vel_.angular.x = 0.0;
        cmd_vel_.angular.y = 0.0;
        cmd_vel_.angular.z = 0.0;
        pub.publish(cmd_vel_);
        ROS_INFO("Changed Success!");
        return;
    // }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_listener");

    talker_listener tl;

    ros::spin();

    return 0;
}

/*
#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Int16MultiArray.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

void slove(short x)
{
    ros::NodeHandle nh;
    std::cout<<x<<std::endl;
    ros::Publisher cmd_vel_wtf;
    cmd_vel_wtf = nh.advertise<geometry_msgs::Twist>("cmd_vell",3);
    geometry_msgs::Twist cmd_vel_;
    x=x>500?500:(x<400?400:x);
    switch (x)
    {
        case 400:
            cmd_vel_.linear.y=0.1;
            break;
        case 500:
            cmd_vel_.linear.y=-0.1;
            break;
        default:
            return ;
    }
    cmd_vel_.linear.x=0.0;
    cmd_vel_.linear.z=0.0;
    cmd_vel_.angular.x=0.0;
    cmd_vel_.angular.y=0.0;
    cmd_vel_.angular.z=0.0;
    cmd_vel_wtf.publish(cmd_vel_);
    ROS_INFO("Changed Success!");
    return ;
}

void poseCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    // ROS_INFO("Color msg:");
    // std::cout << msg << std::endl;
    ROS_INFO("Color data:");
    short bok=0,bok_=0;
    for(int i=0;i<=4;i++)
    {
        std::cout<<msg->data[i]<<' ';
        bok=bok+(msg->data[i]==-1?1:0);
        bok_=(msg->data[i]!=-1?i:bok_);
    }
    std::cout<<std::endl;

    if(bok==4)
    {
        slove(msg->data[bok_]);
    }
    return ;
}

int main(int argc,char** argv)
{
    //初始化myctrl
    ros::init(argc,argv,"myctrl");
    ros::NodeHandle nh;
    ROS_INFO("Start to listen!");
    ros::Subscriber msg_sub = nh.subscribe("/chatter",5 , poseCallback);
    ROS_INFO("wait.");
    ros::spin();
    return 0;
}


#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// 建议将发布者移到这里
ros::Publisher cmd_vel_pub;

void init_publishers()
{
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vell", 3);
}

void slove(short x)
{
    // 移除这里的发布者创建

    geometry_msgs::Twist cmd_vel_;
    x = x > 500 ? 500 : (x < 400 ? 400 : x);
    switch (x)
    {
        case 400:
            cmd_vel_.linear.y = 0.1;
            break;
        case 500:
            cmd_vel_.linear.y = -0.1;
            break;
        default:
            return;
    }
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.z = 0.0;
    cmd_vel_.angular.x = 0.0;
    cmd_vel_.angular.y = 0.0;
    cmd_vel_.angular.z = 0.0;
    
    cmd_vel_pub.publish(cmd_vel_);
    ROS_INFO("Changed Success!");
    return;
}

void poseCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    // ROS_INFO("Color msg:");
    // std::cout << msg << std::endl;
    ROS_INFO("Color data:");
    short bok=0,bok_=0;
    for(int i=0;i<=4;i++)
    {
        std::cout<<msg->data[i]<<' ';
        bok=bok+(msg->data[i]==-1?1:0);
        bok_=(msg->data[i]!=-1?i:bok_);
    }
    std::cout<<std::endl;

    if (bok == 4)
    {
        slove(msg->data[bok_]);
    }
    return;
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "myctrl");
    ROS_INFO("Start to listen!");

    // 初始化发布者
    init_publishers();

    ros::Subscriber msg_sub = nh.subscribe("/chatter", 5, &poseCallback);
    ROS_INFO("wait.");

    // 只在主函数末尾调用ros::spin()
    ros::spin();
    return 0;
}
*/