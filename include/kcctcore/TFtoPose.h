
/**
* @file TFtoPose.cpp
* @brief TF to Pose,PoseStamped and TransformStamped
* @author Shunya Hara
* @date 2021.3.7
* @details TFを定期的に取得してPose,PoseStamped,TransformStampedに変換する
*/

#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <turtlesim/Spawn.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

class TFtoPose{
public:

    TFtoPose(std::string& base_id, std::string& child_id, double rate=10.0);

    geometry_msgs::Pose toPose();
    geometry_msgs::PoseStamped toPoseStamped();
    geometry_msgs::TransformStamped toTransformStamped();

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    std::string base_id_,child_id_;
    geometry_msgs::TransformStamped tfstamped;
    geometry_msgs::PoseStamped posestamped;
};




TFtoPose::TFtoPose(std::string& base_id, std::string& child_id, double rate) : nh_(), tfBuffer_(), tfListener_(tfBuffer_){
    base_id_=base_id;
    child_id_=child_id;
    double duration = 1.0/rate;
    timer_ = nh_.createTimer(ros::Duration(duration), [&](const ros::TimerEvent& e) {
      
      try
      {
        tfstamped = tfBuffer_.lookupTransform(base_id, child_id, ros::Time(0), ros::Duration(0.2));
        posestamped.header=tfstamped.header;
        posestamped.header.frame_id=base_id;
        posestamped.pose.position.x=tfstamped.transform.translation.x;
        posestamped.pose.position.y=tfstamped.transform.translation.y;
        posestamped.pose.position.z=tfstamped.transform.translation.z;
        posestamped.pose.orientation=tfstamped.transform.rotation;
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
        return;
      }
      
    });

}

geometry_msgs::Pose TFtoPose::toPose(){
    return posestamped.pose;
}

geometry_msgs::PoseStamped TFtoPose::toPoseStamped(){
    return posestamped;
}

geometry_msgs::TransformStamped TFtoPose::toTransformStamped(){
    return tfstamped;
}

