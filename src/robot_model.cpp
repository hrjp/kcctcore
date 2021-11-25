#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include"body_model_pub.h"



int main(int argc, char **argv){
    
     ros::init(argc, argv, "robot_model_node");
     ros::NodeHandle n;
     //制御周期10ms
     ros::Rate loop_rate(10);
     body_model body;
     while (n.ok())  {
          body.update();






          ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
          loop_rate.sleep();
    }
    return 0;
}
