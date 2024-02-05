#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdlib.h>

#define SQUARE_SUM(x,y,z) x*x + y*y + z*z
using namespace std;

static tf::Transform map2odom_transform;
static ros::Publisher itemPosePub;
tf::TransformListener* listener1;
geometry_msgs::PoseWithCovariance lastCam2TagPose;
bool firstPass = false;

void GetEulerFromOrientation(const geometry_msgs::Quaternion& orientation){

    tf::Quaternion q;
    tf::quaternionMsgToTF(orientation,q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    roll = roll/3.14 *180;
    pitch = pitch/3.14 * 180;
    yaw = yaw/3.14 * 180;
    cout<<"Roll: "<<roll<<" Pitch: "<<pitch<<" Yaw: "<<yaw<<endl;
}


void map2odomInit(){
    map2odom_transform.setOrigin(tf::Vector3(0.0,0.0,0));
    tf::Quaternion init_quaternion;
    init_quaternion.setRPY(0.0,0.0,0);
    map2odom_transform.setRotation(init_quaternion);
}

void map2odomSet(){
    map2odom_transform.setOrigin(tf::Vector3(map2odom_transform.getOrigin().getX(), map2odom_transform.getOrigin().getY(), 0.0));
    tf::Quaternion safe_quaternion = map2odom_transform.getRotation();
    tf::Matrix3x3 m(safe_quaternion);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    safe_quaternion.setRPY(0.0, 0.0, yaw); // roll pitch 手动置零,initial(0,0,yaw)
    map2odom_transform.setRotation(safe_quaternion);
}

void tagCallback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr &_msg)
{ 

  if(_msg->detections.size()>0){
     vector <apriltags2_ros::AprilTagDetection> map_vect;
     //separate tag to map_tag and item_tag
     for(size_t i = 0; i < _msg->detections.size(); i++){
         if(_msg->detections[i].id[0] < 100){
            
             if(firstPass && abs(_msg->detections[i].pose.pose.pose.position.z - lastCam2TagPose.pose.position.z) > 0.15)   //剔除跳动tf,2格位阈值
             {
                cout<<_msg->detections[i].pose.pose.pose.position.z<<endl;
                 cout<<"delta Z: "<<abs(_msg->detections[i].pose.pose.pose.position.z - lastCam2TagPose.pose.position.z)<<endl;
                 continue;
             }
             map_vect.push_back(_msg->detections[i]);
         }
     }

     cout<<"mapTagNum: "<<map_vect.size()<<endl;
     //process for map_tag
     if(!map_vect.empty()){
        //对map_vect向量中的元素进行由距离camera的大小由小到大排序，
         if(map_vect.size() > 1){
             for(size_t i = 0; i < map_vect.size() -1 ; i++)
                 if(SQUARE_SUM(map_vect[i].pose.pose.pose.position.x, map_vect[i].pose.pose.pose.position.y, map_vect[i].pose.pose.pose.position.z) > SQUARE_SUM(map_vect[i+1].pose.pose.pose.position.x, map_vect[i+1].pose.pose.pose.position.y, map_vect[i+1].pose.pose.pose.position.z))
                     swap(map_vect[i],map_vect[i+1]);
         }
         firstPass = true;
         int tag_num =map_vect[0].id[0];
         cout<<"mapTagID: "<<tag_num<<endl;
         stringstream ss;
         ss << tag_num;
         string tagID = ss.str();

         geometry_msgs::PoseWithCovariance tag_pose;
         tag_pose=map_vect[0].pose.pose;
         lastCam2TagPose = tag_pose;
         tf::Vector3 tag_position;
         tf::Quaternion tag_quaternion(tag_pose.pose.orientation.x,tag_pose.pose.orientation.y,tag_pose.pose.orientation.z,tag_pose.pose.orientation.w);
         tag_position[0] = tag_pose.pose.position.x;
         tag_position[1] = tag_pose.pose.position.y;
         tag_position[2] = tag_pose.pose.position.z;
         //GetEulerFromOrientation(tag_pose.pose.orientation);

         //cam_to_tag(parent_to_child)
         tf::Transform cam2tag_transform, tag2cam_transform;
         cam2tag_transform.setOrigin(tag_position);
         cam2tag_transform.setRotation(tag_quaternion);
         tag2cam_transform = cam2tag_transform.inverse();//逆变换，从标签坐标系变换回摄像头坐标系。
         double x1 = tag2cam_transform.getOrigin().x();
         double y1 = tag2cam_transform.getOrigin().y();
         double z1 = tag2cam_transform.getOrigin().z();
         cout<<"tag2cam_transform_x:"<<x1<<endl;
         cout<<"tag2cam_transform_y:"<<y1<<endl;
         cout<<"tag2cam_transform_z:"<<z1<<endl;

         //odom->cam(parent_to_child)
         tf::StampedTransform transform1;
         auto start_time = ros::Time::now();
         try
         {
             listener1->waitForTransform("/camera_frame",  "/odom",    ros::Time(0), ros::Duration(0.5));
                                                            //parent                 child
             listener1->lookupTransform( "/camera_frame", "/odom",     ros::Time(0), transform1);
         }
         catch (tf::TransformException &ex)
         {
             ROS_ERROR("%s",ex.what());
             ros::Duration(1.0).sleep();
         }
         //cout<<"wait odom2camTF timeuse: "<<( ros::Time::now() - start_time)<<endl;
         tf::Transform cam2odom_transform;
         cam2odom_transform=transform1;

         //tag->map(parent_to_child)
         tf::StampedTransform transform2;
         auto start_time2 =  ros::Time::now();
         try
         {
             listener1->waitForTransform("/map", "tag_" + tagID  ,   ros::Time(0), ros::Duration(1.0));
             //parent    child
             listener1->lookupTransform("/map", "tag_" + tagID,   ros::Time(0), transform2);
            
         }
         catch (tf::TransformException &ex)
         {
             ROS_ERROR("%s",ex.what());
             ros::Duration(1.0).sleep();
         }
         tf::Transform map2tag_transform;
         map2tag_transform=transform2;
         //cout<<"wait tag2mapTF timeuse: "<<( ros::Time::now() - start_time2)<<endl;

         // update
         map2odom_transform =  map2tag_transform * tag2cam_transform  *  cam2odom_transform ;
         
     }
  }
    else
        cout<<"no tag detected!!!"<<endl;
}
 
int main (int argc, char** argv)
{
    ros::init(argc, argv, "odom_calib");
    ros::NodeHandle nh1;
    static tf::TransformBroadcaster br;
    map2odomInit();
    itemPosePub = nh1.advertise<geometry_msgs::PoseStamped>("/item_pose", 1);
    ros::Subscriber sub2 = nh1.subscribe("/tag_detections", 1, tagCallback);
    listener1 = new tf::TransformListener();
    ros::Rate rate(30);
    while (nh1.ok()){
        map2odomSet();
        br.sendTransform(tf::StampedTransform(map2odom_transform, ros::Time::now(),"map","odom"));
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

