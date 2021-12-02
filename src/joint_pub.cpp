/**
* @file joint_pub.cpp
* @brief 
* @author Shunya Hara
* @date 2021.12.2
* @details robot_state_publisherにwheel の角度を送信
* @ref https://qiita.com/srs/items/3eb58fd32eee4d84a530
*/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_pub");
  ros::NodeHandle nh;
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    sensor_msgs::JointState js0;
    js0.header.stamp = ros::Time::now();
    js0.header.seq=count;
    js0.name.resize(2);
    js0.name[0] = "left_wheel_joint";
    js0.name[1] = "right_wheel_joint";
    js0.position.resize(2);
    js0.position[0] = 0.0;
    js0.position[1] = 0.0;
    js0.velocity.resize(2);
    js0.velocity[0] = 0.0;
    js0.velocity[1] = 0.0;
    js0.effort.resize(2);
    js0.effort[0] = 0.0;
    js0.effort[1] = 0.0;
    joint_pub.publish(js0);
    count++;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
