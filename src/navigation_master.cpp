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
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "kcctcore_t/tf_position.h"
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

geometry_msgs::PoseStamped targetWpPose;
void targetWpPose_callback(const geometry_msgs::PoseStamped& poseStamp_message)
{
    targetWpPose = poseStamp_message;
}

geometry_msgs::Pose targetPose;
void targetPose_callback(const geometry_msgs::Pose& pose_message)
{
    targetPose = pose_message;
}

std_msgs::String now_type;
void now_type_callback(const std_msgs::String& now_type_)
{
    now_type=now_type_;
}

double quat2yaw(geometry_msgs::Quaternion orientation)
{
    double roll, pitch, yaw;
    tf::Quaternion quat;
    quaternionMsgToTF(orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference

    return yaw;
}

double arrangeAngle(double angle)
{
    while(angle>M_PI)
    {
        angle -= 2*M_PI;
    }
    while(angle<-M_PI)
    {
        angle += 2*M_PI;
    }

    return angle;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "path_tracking_node");
    ros::NodeHandle n;
   

    //param setting
    ros::NodeHandle pn("~");

    std::string map_id, base_link_id;
    pn.param<std::string>("map_frame_id", map_id, "map");
    pn.param<std::string>("base_link_frame_id", base_link_id, "base_link");
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
    ros::Subscriber recovery_cmd_sub = lSubscriber.subscribe("recovery/cmd_vel", 10, recovery_cmd_callback);
    ros::Subscriber mode_sub = lSubscriber.subscribe("mode_select/mode", 10 , mode_callback);
    ros::Subscriber recovery_mode_sub = lSubscriber.subscribe("recovery/mode", 10 , recovery_mode_callback);
    ros::Subscriber targetWpPose_sub = lSubscriber.subscribe("twist_maneger/targetWpPose_in", 50, targetWpPose_callback);
    ros::Subscriber targetPose_sub = lSubscriber.subscribe("twist_maneger/targetPose_in", 10 , targetPose_callback);
    ros::Subscriber now_type_sub = lSubscriber.subscribe("waypoint/now_type", 10, now_type_callback);

    //cmd_vel publisher
    ros::Publisher cmd_pub=n.advertise<geometry_msgs::Twist>("selected_cmd_vel", 1);
    ros::Publisher mode_pub = n.advertise<std_msgs::String>("mode", 10);

    ros::Publisher yolo_pub = n.advertise<std_msgs::Bool>("enable_yolo", 10);

    tf_position nowPosition(map_id, base_link_id, looprate);
    mode.data = robot_status_str(robot_status::stop);

    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Twist zero_vel;
    zero_vel.linear.x=0.0;
    zero_vel.angular.z=0.0;
    bool pause_mode=true;
    
    bool run_init = true;
    bool recovery_init = false;
    int wp_stack=0;

    while (n.ok()){

        if(button_clicked==buttons_status_start){
            pause_mode=false;
            mode.data = robot_status_str(robot_status::run);

        }
        if(button_clicked==buttons_status_pause){
            pause_mode=true;
            mode.data = robot_status_str(robot_status::stop);
        }

        cmd_vel=mcl_cmd_vel;

        if(run_init){
            //run init mode
            if(mode.data == robot_status_str(robot_status::run)){
                double dx = targetPose.position.x - nowPosition.getPose().position.x;
                double dy = targetPose.position.y - nowPosition.getPose().position.y;
                double targetAngle = atan2(dy, dx);
                double diffAngle = arrangeAngle(targetAngle - nowPosition.getYaw());

                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = diffAngle * 1.5;
                if(abs(diffAngle) < 10*M_PI/180){
                    run_init = false;
                }
            }
        }

        //stop
        if(mode.data == robot_status_str(robot_status::stop)){
            cmd_vel=zero_vel;
            run_init = true;
        }


        //angle adjust
        if(mode.data == robot_status_str(robot_status::angleAdjust)){
            double diffAngle = arrangeAngle(quat2yaw(targetWpPose.pose.orientation) - nowPosition.getYaw());

            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = diffAngle * 1.5;
            if(abs(diffAngle) < 1*M_PI/180){
                mode.data = robot_status_str(robot_status::stop);
            }
        }
        if(recovery_mode.data == robot_status_str(robot_status::safety_stop)){
            mode.data = robot_status_str(robot_status::safety_stop);
        }
        if(recovery_mode.data == robot_status_str(robot_status::run) && mode.data == robot_status_str(robot_status::safety_stop)){
            mode.data = robot_status_str(robot_status::run);
        }

        //safety stop
        if(recovery_mode.data == robot_status_str(robot_status::safety_stop)){
            mode.data = robot_status_str(robot_status::safety_stop);
        }
        //end safety stop
        if(recovery_mode.data == robot_status_str(robot_status::run) && mode.data == robot_status_str(robot_status::safety_stop)){
            mode.data = robot_status_str(robot_status::run);
        }

        //recovery mode
        if(recovery_mode.data == robot_status_str(robot_status::recovery)){
            recovery_init = true;
            mode.data = robot_status_str(robot_status::recovery);
            cmd_vel = recovery_cmd_vel;
        }
        if(recovery_init){
            if(!(recovery_mode.data == robot_status_str(robot_status::recovery))){
                recovery_init = false;

                run_init = true;
                mode.data = robot_status_str(robot_status::run);
            }
        }


        //人検出
        static bool person_mode=false;
        static bool person_mode_once=true;
        static bool person_angle=false;
        static bool enable_yolo=false;
        if(now_type.data==waypoint_type_str(waypoint_type::person_detection)){
            if(person_mode_once){
                person_mode=true;
                person_mode_once=false;
                person_angle=true;
            }
        }
        else{
            person_mode_once=true;
            
        }
        if(person_mode){
            // start buttonで通常走行に復帰
            if(button_clicked==buttons_status_start){
                person_mode=false;
                enable_yolo=false;
            }
            //角度を合わせてから追従開始
            else if(person_angle){
                double dx = targetPose.position.x - nowPosition.getPose().position.x;
                double dy = targetPose.position.y - nowPosition.getPose().position.y;
                double targetAngle = atan2(dy, dx);
                double diffAngle = arrangeAngle(targetAngle - nowPosition.getYaw());

                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = diffAngle * 1.5;
                if(abs(diffAngle) < 10*M_PI/180){
                    person_angle = false;
                }
            }
            else{
                cmd_vel.linear.x=camera_cmd_vel.linear.x;
                cmd_vel.angular.z=camera_cmd_vel.angular.z;
                enable_yolo=true;
            }
            
        }

        std_msgs::Bool enable_yolo_msg;
        enable_yolo_msg.data=enable_yolo;
        yolo_pub.publish(enable_yolo_msg);

        cmd_pub.publish(cmd_vel);
        mode_pub.publish(mode);

        button_clicked=buttons_status_free;
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();

    }
    
    return 0;
}
