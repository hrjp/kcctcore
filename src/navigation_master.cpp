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
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <kcctcore/button_status.h>
#include <kcctcore/waypoint_type.h>

using namespace std;

geometry_msgs::Twist mcl_cmd_vel;
void mcl_cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_){
    mcl_cmd_vel=cmd_vel_;
}

geometry_msgs::Twist camera_cmd_vel;
void camera_cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_){
    camera_cmd_vel=cmd_vel_;
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

    double looprate=100.0;
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
    
    //cmd_vel publisher
    ros::Publisher cmd_pub=n.advertise<geometry_msgs::Twist>("selected_cmd_vel", 1);

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
            cout<<"camera_mode"<<endl;
        }
        else{
            cmd_vel=mcl_cmd_vel;
        }
        
        if(pause_mode){
            cmd_vel=zero_vel;
        }

        cmd_pub.publish(cmd_vel);




        button_clicked=buttons_status_free;
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }
    
    return 0;
}
