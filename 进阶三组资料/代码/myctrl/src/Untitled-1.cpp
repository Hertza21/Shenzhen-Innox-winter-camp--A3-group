
#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Int16MultiArray.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

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
        pub = nh.advertise<geometry_msgs::Twist>("cmd_vell", 5);
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
    if (bok == 4)
    {
        short x = x > 500 ? 500 : (x < 400 ? 400 : x);
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
        pub.publish(cmd_vel_);
        ROS_INFO("Changed Success!");
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_listener");

    talker_listener tl;

    ros::spin();

    return 0;
}
