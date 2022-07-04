/**
* @file navigation_master.cpp
* @brief 
* @author Shunya Hara
* @date 2021.3.11
* @details ナビゲーションのマスター司令ノード
*/

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Pose.h>

#include "kcctcore/robot_state.h"
#include "kcctcore/button_state.h"
#include "kcctcore/waypoint_type.h"
#include "kcctcore_t/tf_position.h"

RobotState _robotState = RobotState::STOP;
void robotState_cb(const std_msgs::String& robotState)
{
    _robotState = String2RobotState(robotState.data);
} // void robotState_cb()

RobotState _robotState_recov = RobotState::STOP;
void robotState_recov_cb(const std_msgs::String& robotState_recov)
{
    _robotState_recov = String2RobotState(robotState_recov.data);
} // void robotState_recov_cb()

ButtonState _buttonState = ButtonState::FREE;
void buttonState_cb(const std_msgs::Int32 buttonState){
    _buttonState = static_cast<ButtonState>(buttonState.data);
    ROS_INFO_STREAM("Input : " << ButtonState2String(_buttonState));
} // void buttonState_cb()

geometry_msgs::Pose _pose_target;
void pose_target_cb(const geometry_msgs::Pose& pose_target)
{
    _pose_target = pose_target;
} // void pose_target_cb()

geometry_msgs::PoseStamped _pose_targetWp;
void pose_targetWp_cb(const geometry_msgs::PoseStamped& pose_targetWp)
{
    _pose_targetWp = pose_targetWp;
} // void pose_targetWp_cb()

geometry_msgs::Twist _cmd_vel_mcl;
void cmd_vel_mcl_cb(const geometry_msgs::Twist& cmd_vel_mcl){
    _cmd_vel_mcl = cmd_vel_mcl;
} // void cmd_vel_mcl_cb()

geometry_msgs::Twist _cmd_vel_recov;
void cmd_vel_recov_cb(const geometry_msgs::Twist& cmd_vel_recov)
{
    _cmd_vel_recov = cmd_vel_recov;
} // void cmd_vel_recov_cb()

geometry_msgs::Twist _cmd_vel_camera;
void cmd_vel_camera_cb(const geometry_msgs::Twist& cmd_vel_camera)
{
    _cmd_vel_camera = cmd_vel_camera;
} // void cmd_vel_camera_cb()

int32_t _wp_now;
void wp_now_cb(const std_msgs::Int32& wp_now)
{
    _wp_now = wp_now.data;
}

WaypointType _wpType;
void wpType_cb(const std_msgs::String& wpType){
    _wpType =  String2WaypointType(wpType.data);
} // void wpType_cb()

double getDeltaAngle(double current, double target)
{
    current -= 360.0 * ((int)(current / 360.0));
    target -= 360.0 * ((int)(target / 360.0));

    if (std::abs(target - current) > 180)
        return target - current - (target - current)>0?1.0:-1.0 * 360.0;
    else
        return target - current;
} // double getDeltaAngle()

double quat2yaw(geometry_msgs::Quaternion orientation)
{
    double roll, pitch, yaw;
    tf::Quaternion quat;
    quaternionMsgToTF(orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
} // double quat2yaw

int main(int argc, char **argv){
    ros::init(argc, argv, "kcctcore/navigation_master");
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");
    ros::NodeHandle lSubscriber("");

    /* Param */

    // from .launch
    std::string frameId_map, frameId_baseLink;
    pn.param<std::string>("map_frame_id", frameId_map, "map");
    pn.param<std::string>("base_link_frame_id", frameId_baseLink, "base_link");
    
    double loopRate;
    pn.param<double>("loop_rate", loopRate, 100.0);
    ros::Rate rate_roop(loopRate);

    // Class and Structure
    tf_position pose_now(frameId_map, frameId_baseLink, loopRate);

    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Twist cmd_vel_zero;
    cmd_vel_zero.linear.x=0.0;
    cmd_vel_zero.angular.z=0.0;

    // Others
    bool is_initState = true;
    bool is_initState_recov = false;

    int32_t wp_recurStart = -1;
    int32_t recursionCount = 0;

    /* Publisher */
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("selected_cmd_vel", 1);
    ros::Publisher robotState_pub = nh.advertise<std_msgs::String>("mode", 10);
    ros::Publisher wpOverride_pub = nh.advertise<std_msgs::Int32>("waypoint/set", 1);
    ros::Publisher yolo_pub = nh.advertise<std_msgs::Bool>("enable_yolo", 10);

    /* Subscriber */
    ros::Subscriber robotState_sub = lSubscriber.subscribe("mode_select/mode", 10 , robotState_cb);
    ros::Subscriber robotState_recov_sub = lSubscriber.subscribe("recovery/mode", 10 , robotState_recov_cb);
    ros::Subscriber buttonState_sub = lSubscriber.subscribe("buttons", 50, buttonState_cb);
    ros::Subscriber wp_now_sub = lSubscriber.subscribe("waypoint/now", 50, wp_now_cb);
    ros::Subscriber wpType_sub = lSubscriber.subscribe("waypoint/now_type", 10, wpType_cb);
    ros::Subscriber cmd_vel_mcl_sub = lSubscriber.subscribe("mcl_cmd_vel", 50, cmd_vel_mcl_cb);
    ros::Subscriber cmd_vel_recov_sub = lSubscriber.subscribe("recovery/cmd_vel", 10, cmd_vel_recov_cb);
    ros::Subscriber cmd_vel_camera_sub = lSubscriber.subscribe("camera_cmd_vel", 50, cmd_vel_camera_cb);
    ros::Subscriber pose_target_sub = lSubscriber.subscribe("twist_maneger/targetPose_in", 10 , pose_target_cb);
    ros::Subscriber pose_targetWp_sub = lSubscriber.subscribe("twist_maneger/targetWpPose_in", 50, pose_targetWp_cb);

    while(nh.ok()){
        
        cmd_vel=_cmd_vel_mcl;

        /* Initialize */
        if(is_initState){
            if(_robotState == RobotState::RUN){
                double dx = _pose_target.position.x -pose_now.getPose().position.x;
                double dy = _pose_target.position.y - pose_now.getPose().position.y;
                double angle_target = atan2(dy, dx);
                double angle_diff = getDeltaAngle(0, angle_target - pose_now.getYaw());
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = angle_diff * 1.5;
                if(abs(angle_diff) < 10*M_PI/180){
                    is_initState = false;
                } // if
            } // if
        } // if

        if(is_initState_recov){
            if(!(_robotState_recov == RobotState::RECOVERY)){
                is_initState_recov = false;
                is_initState = true;
                _robotState = RobotState::RUN;
            } // if
        } // if

        /* Input */
        switch(_buttonState){
            case ButtonState::START:
                _robotState = RobotState::RUN;
                break;
            case ButtonState::PAUSE:
                _robotState = RobotState::STOP;
                break;
        } // switch(ButtonState)

        /* Movement */
        switch(_robotState){
            case RobotState::STOP:
                cmd_vel = cmd_vel_zero;
                is_initState = true;
                break;
            case RobotState::ADJUST_ANGLE:
                double angle_diff = getDeltaAngle(0, quat2yaw(_pose_targetWp.pose.orientation) - pose_now.getYaw());
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = angle_diff * 1.5;
                if(abs(angle_diff) < 1.0*M_PI/180)
                    _robotState = RobotState::STOP;
               break;
        } // switch(RobotState)

        switch(_robotState_recov){
            case RobotState::SAFETY_STOP:
                _robotState = RobotState::SAFETY_STOP;
                break;
            case RobotState::RUN:
                if(_robotState == RobotState::SAFETY_STOP)
                    _robotState = RobotState::RUN;
                break;
            case RobotState::RECOVERY:
                is_initState_recov = true;
                _robotState = RobotState::RECOVERY;
                cmd_vel = _cmd_vel_recov;
                break;
        } // switch(RobotState)

        /* WaypointType */
        switch(_wpType){
            case WaypointType::recursion_start:
                /* YoloStart */
                wp_recurStart = _wp_now;
                recursionCount = 0;
                break;
            case WaypointType::recursion_end:
                /* YoloStop */
                if(wp_recurStart == -1) break;
                if(recursionCount > 2)
                {
                    ROS_INFO("RECURSION END");
                    wp_recurStart = -1;
                }
                else
                {
                    std_msgs::Int32 msg;
                    msg.data = wp_recurStart + 1;
                    wpOverride_pub.publish(msg);
                    recursionCount++;
                }
                break;
        } // switch(WaypointType)

        /* Publish */
        cmd_vel_pub.publish(cmd_vel);
        std_msgs::String robotState_msg;
        robotState_msg.data = RobotState2String(_robotState);
        robotState_pub.publish(robotState_msg);

        /* Button Release*/
        _buttonState = ButtonState::FREE;
        
        /* Spin */
        ros::spinOnce();
        rate_roop.sleep();
    } // while
    return 0;
} // int main()