#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Int16MultiArray.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>

#define speeded 0.02
#define uppered 700
#define lowered 600

double bok=2; // 状态为0时->正在前进;状态为1时->刚结束前进并等待矫正方位;状态为2时->已完成方位矫正
bool ans=false;
geometry_msgs::Quaternion qt;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    if(bok==0 && msg->linear.z==1)
    {
        std::cout<<"-------------------------------------"<<std::endl;
        bok = 1;
    }
}

void odomVelCallback(const nav_msgs::OdometryConstPtr& msg)
{
    if(bok==1)
    {
        ans=true;
        qt=msg->pose.pose.orientation;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("cmd_vel", 5,  &cmdVelCallback);
    ros::Subscriber sub_odom= nh.subscribe("odom",5, &odomVelCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 5);
    ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("cmd_vel", 2);
    
    ros::Rate looprate(10);

    nav_msgs::Odometry position;
    tf::Quaternion qt1;

    double roll, pitch, yaw,xx,yy;
    while (ros::ok())
    {
        ros::spinOnce();
        looprate.sleep();
        if(bok==2)
        {
            std::cout<<"please input:"<<std::endl;
            std::cin>>xx>>yy;
            bok=-1,ans=false;
        }
        if(bok==1&&ans==true)
        {
            // ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 3);
            std::cout<<"test started......"<<std::endl;
            tf::quaternionMsgToTF(qt,qt1);
            // tf::Quaternion tf_quat(qt.x, qt.y, qt.z, qt.w);
            // tf::Matrix3x3 rot_matrix(tf_quat); // 获取欧拉角
            tf::Matrix3x3(qt1).getRPY(roll,pitch,yaw);
            // 从旋转矩阵获取欧拉角（RPY）
            // q=createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
            //rot_matrix.getRPY(roll, pitch, yaw);
            // std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
            std::cout<< "Yaw: " << yaw << std::endl;
            geometry_msgs::Twist cmd_vel_;
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.linear.z = 0.0;
            cmd_vel_.angular.x = 0.0;
            cmd_vel_.angular.y = 0.0;
            cmd_vel_.angular.z = 0.0;
            double a=yaw>0?yaw:-1.0*yaw;
            double yaw_speed=0.03;
            if(a>0.02)
            {
                if(a>0.1)
                    yaw_speed=0.1;
                if(a>0.3)
                    yaw_speed=0.25;
                if(a>0.5)
                    yaw_speed=0.4;
                if(yaw>0)
                {
                    cmd_vel_.angular.z= -1.0*yaw_speed;
                }
                else if(yaw<0)
                {
                    cmd_vel_.angular.z= yaw;
                }
                }
            else
            {
                cmd_vel_.angular.z=0.0;
                bok=2;
            }
            pub1.publish(cmd_vel_);
            // ans=false;
        }
        if(bok==-1)
        {
            for(int i=0;i<=2;i++)
            {
                geometry_msgs::PointStamped point_msg;
                //填充header信息
                point_msg.header.stamp=ros::Time::now();
                point_msg.header.frame_id="map";

                //填充点的三位坐标信息
                point_msg.point.x=xx;
                point_msg.point.y=yy;
                point_msg.point.z=0.0;
                //发布消息
                pub.publish(point_msg);
                std::cout<<"Send point..."<<std::endl;
            }
            bok=0;
        }
        std::cout<<bok<<std::endl;
    }
    // ros::spin();
    std::cout<<"direction dead..."<<std::endl;
    return 0;
}
/*
#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Int16MultiArray.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>

#define speeded 0.02
#define uppered 700
#define lowered 600

double bok=2; // 状态为0时->正在前进;状态为1时->刚结束前进并等待矫正方位;状态为2时->已完成方位矫正
double xx,yy;

class talker_listener
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber sub_odom;
    ros::Publisher pub;

public:
    talker_listener(ros::NodeHandle &given_nh):nh(given_nh)
    {
        // std::cout<<"Try to send point..."<<std::endl;
        
        // Test
        
        
        sub = nh.subscribe("cmd_vel", 50,  &talker_listener::cmdVelCallback, this);
        sub_odom = nh.subscribe("odom",50, &talker_listener::odomVelCallback, this);
        // std::cout<<bok<<std::endl;
    }
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void odomVelCallback(const nav_msgs::OdometryConstPtr& msg);
};


void talker_listener::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    std::cout<<"-------------------------------------"<<std::endl;
    if(bok==0 && msg->linear.z==1)
    {    
        bok = 1;
    }
}

void talker_listener::odomVelCallback(const nav_msgs::OdometryConstPtr& msg)
{
    
    if(bok!=1)
        return ;
    double roll, pitch, yaw;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 3);
    std::cout<<"test started......"<<std::endl;
    do
    {
        geometry_msgs::Quaternion qt=msg->pose.pose.orientation;
        tf::Quaternion tf_quat(qt.x, qt.y, qt.z, qt.w);
        tf::Matrix3x3 rot_matrix(tf_quat); // 获取欧拉角
        // 从旋转矩阵获取欧拉角（RPY）
        rot_matrix.getRPY(roll, pitch, yaw);
        // std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
        std::cout<< "Yaw: " << yaw << std::endl;
        geometry_msgs::Twist cmd_vel_;
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.linear.y = 0.0;
        cmd_vel_.linear.z = 0.0;
        cmd_vel_.angular.x = 0.0;
        cmd_vel_.angular.y = 0.0;
        cmd_vel_.angular.z = 0.0;
        if(yaw>0)
        {
            cmd_vel_.angular.z= -0.1;
        }
        else if(yaw<0)
        {
            cmd_vel_.angular.z= 0.1;
        }
        pub.publish(cmd_vel_);
    } while((yaw>0?yaw:-1.0*yaw)<0.1);
    bok=2;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_listener");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 3);
    ros::Rate looprate(10);
    while (ros::ok())
    {
        if(bok==2)
        {
            std::cout<<"please input:"<<std::endl;
            std::cin>>xx>>yy;
            bok=0;
        }
        if(bok==0)
        {
            geometry_msgs::PointStamped point_msg;
            //填充header信息
            point_msg.header.stamp=ros::Time::now();
            point_msg.header.frame_id="map";

            //填充点的三位坐标信息
            point_msg.point.x=xx;
            point_msg.point.y=yy;
            point_msg.point.z=0.0;
            //发布消息
            pub.publish(point_msg);
            std::cout<<"Send point..."<<std::endl;
        }
        talker_listener tl(nh);
        ros::spinOnce();
        looprate.sleep();
        // std::cout<<bok<<std::endl;
    }
    // ros::spin();
    std::cout<<"direction dead..."<<std::endl;
    return 0;
}
*/
/*
#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Int16MultiArray.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>

#define speeded 0.02
#define uppered 700
#define lowered 600

double bok=0; // 状态为0时->正在前进;状态为1时->刚结束前进并等待矫正方位;状态为2时->已完成方位矫正
double xx,yy;

class talker_listener
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber sub_odom;
    ros::Publisher pub;

public:
    talker_listener()
    {
        pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 3);
        std::cout<<"Try to send point..."<<std::endl;
        
        // Test
        ros::Rate looprate(10);
        while(ros::ok())
        {
            geometry_msgs::PointStamped point_msg;
            //填充header信息
            point_msg.header.stamp=ros::Time::now();
            point_msg.header.frame_id="map";

            //填充点的三位坐标信息
            point_msg.point.x=xx;
            point_msg.point.y=yy;
            point_msg.point.z=0.0;
            //发布消息
            pub.publish(point_msg);
            std::cout<<"Send point..."<<std::endl;
            
            ros::spinOnce();
            looprate.sleep();
        }
        
        std::cout<<bok<<std::endl;
        sub = nh.subscribe("cmd_vel", 5,  &talker_listener::cmdVelCallback, this);
        // sub_odom = nh.subscribe("odom",2, &talker_listener::odomVelCallback, this);
        std::cout<<bok<<std::endl;
    }
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void odomVelCallback(const nav_msgs::OdometryConstPtr& msg);
};

// void talker_listener::PointCallback(const )


void talker_listener::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    if(bok==0 && msg->linear.z==1)
    {    
        bok = 1;
    }
}

void talker_listener::odomVelCallback(const nav_msgs::OdometryConstPtr& msg)
{
    if(bok!=1)
        return ;
    double roll, pitch, yaw;
    std::cout<<"test started......"<<std::endl;
    do
    {
        geometry_msgs::Quaternion qt=msg->pose.pose.orientation;
        tf::Quaternion tf_quat(qt.x, qt.y, qt.z, qt.w);
        tf::Matrix3x3 rot_matrix(tf_quat); // 获取欧拉角
        // 从旋转矩阵获取欧拉角（RPY）
        rot_matrix.getRPY(roll, pitch, yaw);
        // std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
        std::cout<< "Yaw: " << yaw << std::endl;
        geometry_msgs::Twist cmd_vel_;
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.linear.y = 0.0;
        cmd_vel_.linear.z = 0.0;
        cmd_vel_.angular.x = 0.0;
        cmd_vel_.angular.y = 0.0;
        cmd_vel_.angular.z = 0.0;
        if(yaw>0)
        {
            cmd_vel_.angular.z= -0.1;
        }
        else if(yaw<0)
        {
            cmd_vel_.angular.z= 0.1;
        }
        pub.publish(cmd_vel_);
    } while((yaw>0?yaw:-1.0*yaw)<0.1);
    bok=2;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_listener");
    while (true)
    {
        std::cout<<"please input:"<<std::endl;
        std::cin>>xx>>yy;
        bok=0;
        talker_listener tl;
    }
    ros::spin();
    std::cout<<"direction dead..."<<std::endl;
    return 0;
}
*/