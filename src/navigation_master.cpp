/**
* @file navigation_master.cpp
* @brief 
* @author Shunya Hara
* @date 2021.3.11
* @details ナビゲーションのマスター司令ノード
*/

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "kcctcore/button_status.h"
#include "kcctcore/waypoint_type.h"
#include "kcctcore/robot_status.h"

using namespace std;

geometry_msgs::Twist mcl_cmd_vel;
void mcl_cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_){
    mcl_cmd_vel=cmd_vel_;
}

geometry_msgs::Twist camera_cmd_vel;
void camera_cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_){
    camera_cmd_vel=cmd_vel_;
}

std_msgs::String mode;
void mode_callback(const std_msgs::String& mode_message)
{
    mode = mode_message;
}

geometry_msgs::Twist recovery_cmd_vel;
void recovery_cmd_callback(const geometry_msgs::Twist& cmd_message)
{
    recovery_cmd_vel = cmd_message;
}

std_msgs::String recovery_mode;
void recovery_mode_callback(const std_msgs::String& mode_message)
{
    recovery_mode = mode_message;
}

int button_clicked=0;
void buttons_callback(const std_msgs::Int32 sub_buttons){
    button_clicked=sub_buttons.data;
}

int now_wp=0;
void now_wp_callback(const std_msgs::Int32& now_wp_){
    now_wp=now_wp_.data;
}

vector<int> wp_type;
void wp_type_callback(const std_msgs::Int32MultiArray& wp_type_){
    wp_type.resize(wp_type_.data.size());
    wp_type=wp_type_.data;
}

nav_msgs::Path path;
void path_callback(const nav_msgs::Path& path_){
    path=path_;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "path_tracking_node");
    ros::NodeHandle n;
   

    //param setting
    ros::NodeHandle pn("~");

    std::string map_id, base_link_id;
    pnh.param<std::string>("map_frame_id", map_id, "map");
    pnh.param<std::string>("base_link_frame_id", base_link_id, "base_link");
    double looprate;
    pn.param<double>("loop_rate",looprate,100.0);

     //制御周期10Hz
    ros::Rate loop_rate(looprate);

    ros::NodeHandle lSubscriber("");

    //cmd_vel subscliber
    ros::Subscriber mcl_cmd_vel_sub = lSubscriber.subscribe("mcl_cmd_vel", 50, mcl_cmd_vel_callback);
    //camera_cmd_vel subsliber
    ros::Subscriber camera_cmd_vel_sub = lSubscriber.subscribe("camera_cmd_vel", 50, camera_cmd_vel_callback);
    //rviz control panel subscliber
    ros::Subscriber buttons_sub = lSubscriber.subscribe("buttons", 50, buttons_callback);
    //waypoint/now subscliber
    ros::Subscriber now_wp_sub = lSubscriber.subscribe("waypoint/now", 50, now_wp_callback);
    //waypoint type
    ros::Subscriber wp_type_sub = lSubscriber.subscribe("waypoint/type", 50, wp_type_callback);
    ros::Subscriber recovery_cmd_sub = nh.subscribe("recovery/cmd_vel", 10, recovery_cmd_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode_select/mode", 10 , mode_callback);
    ros::Subscriber recovery_mode_sub = nh.subscribe("recovery/mode", 10 , recovery_mode_callback);
    
    //cmd_vel publisher
    ros::Publisher cmd_pub=n.advertise<geometry_msgs::Twist>("selected_cmd_vel", 1);
    ros::Publisher mode_pub = nh.advertise<std_msgs::String>("mode", 10);

    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Twist zero_vel;
    zero_vel.linear.x=0.0;
    zero_vel.angular.z=0.0;
    bool pause_mode=true;
    bool person_mode=false;
    int wp_stack=0;

    while (n.ok())  {

        if(button_clicked==buttons_status_start){
            pause_mode=false;
            person_mode=false;
        }
        if(button_clicked==buttons_status_pause){
            pause_mode=true;
        }

        if(wp_stack<now_wp){
            if(wp_type.at(now_wp)==waypoint_type_camera){
                std::cout<<"camera mode"<<std::endl;
                person_mode=true;
                wp_stack=now_wp;
            }
            if(wp_type.at(now_wp)==waypoint_type_stop){
                pause_mode=true;
                wp_stack=now_wp;
            }
        }


        if(person_mode){
            cmd_vel=camera_cmd_vel;
            std::cout<<"camera_mode"<<std::endl;
        }
        else{
            cmd_vel=mcl_cmd_vel;
        }
        
        if(pause_mode){
            cmd_vel=zero_vel;
        }


        //angle adjust
        if(mode.data == STR(robot_status::angleAdjust)){
            double diffAngle = arrangeAngle(quat2yaw(targetWpPose.pose.orientation) - nowPosition.getYaw());

            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = constrain(diffAngle * 1.5, -max_angular_vel, max_angular_vel);
            if(abs(diffAngle) < 1*M_PI/180){
                mode.data = STR(robot_status::stop);
            }
        }
        if(recovery_mode.data == STR(robot_status::safety_stop)){
            mode.data = STR(robot_status::safety_stop);
        }
        if(recovery_mode.data == STR(robot_status::run) && mode.data == STR(robot_status::safety_stop)){
            mode.data = STR(robot_status::run);
        }

        //recovery mode
        if(recovery_mode.data == STR(robot_status::recovery)){
            recovery_init = true;
            mode.data = STR(robot_status::recovery);
            cmd_vel = recovery_cmd_vel;
        }
        if(recovery_init){
            if(!(recovery_mode.data == STR(robot_status::recovery))){
                recovery_init = false;

                run_init = true;
                mode.data = "run";
            }
        }

        cmd_pub.publish(cmd_vel);




        button_clicked=buttons_status_free;
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}
