#include <ros/ros.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include"geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <iostream>

void sendTransform(tf2_ros::StaticTransformBroadcaster &pub, int id, double x, double y){
    geometry_msgs::TransformStamped ts;
    ts.header.seq = 100;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "map";
   
    ts.child_frame_id = "tag_" + std::to_string(id);
    ts.transform.translation.x = x;
    ts.transform.translation.y = y;
    ts.transform.translation.z = 0;

    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();
    
    pub.sendTransform(ts);
}
int main(int argc, char  *argv[]){
    ros::init(argc,argv,"static_tf");
    ros::NodeHandle nh;
    tf2_ros::StaticTransformBroadcaster pub ;


    int id = 000;
    for(int i=0;i<5;i++){
        sendTransform(pub, id, 0.25+i*0.5, 3.25);
        id++;
    }

    for(int i=0;i<5;i++){
        sendTransform(pub, id, 4.75+i*0.5, 3.25);
        id++;
    }
    
    
    for(int i=0;i<5;i++){
        sendTransform(pub, id, 0.25+i*0.5, 2.75);
        id++;
    }
    for(int i=0;i<2;i++){
        sendTransform(pub, id, 3.25+i*0.5, 2.75);
        id++;
    }
    for(int i=0;i<5;i++){
        sendTransform(pub, id, 4.75+i*0.5, 2.75);
        id++;
    }
  

  

    for(int i=0;i<3;i++){
        sendTransform(pub, id, 1.25+i*0.5, 2.25);
        id++;
    }

    for(int i=0;i<3;i++){
        sendTransform(pub, id, 4.75+i*0.5, 2.25);
        id++;
    }

    for(int i=0;i<3;i++){
        sendTransform(pub, id, 1.25+i*0.5, 1.25);
        id++;
    }

    for(int i=0;i<3;i++){
        sendTransform(pub, id, 4.75+i*0.5, 1.25);
        id++;
    }

    
    
    for(int i=0;i<5;i++){
        sendTransform(pub, id, 0.25+i*0.5, 0.75);
        id++;
    }
    for(int i=0;i<2;i++){
        sendTransform(pub, id, 3.25+i*0.5, 0.75);
        id++;
    }
    for(int i=0;i<5;i++){
        sendTransform(pub, id, 4.75+i*0.5, 0.75);
        id++;
    }
  

    for(int i=0;i<5;i++){
        sendTransform(pub, id, 0.25+i*0.5, 0.25);
        id++;
    }

    for(int i=0;i<5;i++){
        sendTransform(pub, id, 4.75+i*0.5, 0.25);
        id++;
    }

    ros::spin();
    return 0;
}
