#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <math.h>
#include <sstream>


using namespace std;
class body_model{
    public:
    body_model();
    void update(char *fix_id);
    private:
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    ros::Publisher marker_pub1;
};


body_model::body_model(){
    marker_pub   = n.advertise<visualization_msgs::MarkerArray>("body_model", 1);
}

void body_model::update(char *fix_id){
    visualization_msgs::MarkerArray marker_array;

    marker_array.markers.resize(10);
        //marker0
    int i=0;
    marker_array.markers[i].header.frame_id = fix_id;
    marker_array.markers[i].header.stamp = ros::Time::now();
    marker_array.markers[i].ns = "cmd_vel_display";
    marker_array.markers[i].id = i;
    marker_array.markers[i].lifetime = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array.markers[i].scale.x = 0.85;
    marker_array.markers[i].scale.y = 0.75;
    marker_array.markers[i].scale.z = 0.1;
    marker_array.markers[i].pose.position.x=-0.2;
    marker_array.markers[i].pose.position.y=0;
    marker_array.markers[i].pose.position.z=0.2;
    marker_array.markers[i].pose.orientation.x=0;
    marker_array.markers[i].pose.orientation.y=0;
    marker_array.markers[i].pose.orientation.z=0;
    marker_array.markers[i].pose.orientation.w=0;
    marker_array.markers[i].color.r = 0.8f;
    marker_array.markers[i].color.g = 0.8f;
    marker_array.markers[i].color.b = 0.8f;
    marker_array.markers[i].color.a = 0.8f;


    i=2;
    marker_array.markers[i].header.frame_id = fix_id;
    marker_array.markers[i].header.stamp = ros::Time::now();
    marker_array.markers[i].ns = "cmd_vel_display";
    marker_array.markers[i].id = i;
    marker_array.markers[i].lifetime = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array.markers[i].scale.x = 0.7;
    marker_array.markers[i].scale.y = 0.45;
    marker_array.markers[i].scale.z = 0.6;
    marker_array.markers[i].pose.position.x=-0.2;
    marker_array.markers[i].pose.position.y=0;
    marker_array.markers[i].pose.position.z=0.55;
    marker_array.markers[i].pose.orientation.x=0;
    marker_array.markers[i].pose.orientation.y=0;
    marker_array.markers[i].pose.orientation.z=0;
    marker_array.markers[i].pose.orientation.w=0;
    marker_array.markers[i].color.r = 0.8f;
    marker_array.markers[i].color.g = 0.8f;
    marker_array.markers[i].color.b = 0.8f;
    marker_array.markers[i].color.a = 0.8f;

     i=3;
    marker_array.markers[i].header.frame_id = fix_id;
    marker_array.markers[i].header.stamp = ros::Time::now();
    marker_array.markers[i].ns = "cmd_vel_display";
    marker_array.markers[i].id = i;
    marker_array.markers[i].lifetime = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
    marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array.markers[i].scale.x = 0.3;
    marker_array.markers[i].scale.y = 0.3;
    marker_array.markers[i].scale.z = 0.1;
    marker_array.markers[i].pose.position.x=0;
    marker_array.markers[i].pose.position.y=0.3;
    marker_array.markers[i].pose.position.z=0.15;
    marker_array.markers[i].pose.orientation.x=1.0;
    marker_array.markers[i].pose.orientation.y=0;
    marker_array.markers[i].pose.orientation.z=0;
    marker_array.markers[i].pose.orientation.w=1.0;
    marker_array.markers[i].color.r = 0.2f;
    marker_array.markers[i].color.g = 0.2f;
    marker_array.markers[i].color.b = 0.2f;
    marker_array.markers[i].color.a = 0.9f;

    i=4;
    marker_array.markers[i].header.frame_id = fix_id;
    marker_array.markers[i].header.stamp = ros::Time::now();
    marker_array.markers[i].ns = "cmd_vel_display";
    marker_array.markers[i].id = i;
    marker_array.markers[i].lifetime = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
    marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array.markers[i].scale.x = 0.3;
    marker_array.markers[i].scale.y = 0.3;
    marker_array.markers[i].scale.z = 0.1;
    marker_array.markers[i].pose.position.x=0;
    marker_array.markers[i].pose.position.y=-0.3;
    marker_array.markers[i].pose.position.z=0.15;
    marker_array.markers[i].pose.orientation.x=1.0;
    marker_array.markers[i].pose.orientation.y=0;
    marker_array.markers[i].pose.orientation.z=0;
    marker_array.markers[i].pose.orientation.w=1.0;
    marker_array.markers[i].color.r = 0.2f;
    marker_array.markers[i].color.g = 0.2f;
    marker_array.markers[i].color.b = 0.2f;
    marker_array.markers[i].color.a = 0.9f;

    i=5;
    marker_array.markers[i].header.frame_id = fix_id;
    marker_array.markers[i].header.stamp = ros::Time::now();
    marker_array.markers[i].ns = "cmd_vel_display";
    marker_array.markers[i].id = i;
    marker_array.markers[i].lifetime = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
    marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array.markers[i].scale.x = 0.15;
    marker_array.markers[i].scale.y = 0.15;
    marker_array.markers[i].scale.z = 0.05;
    marker_array.markers[i].pose.position.x=-0.485;
    marker_array.markers[i].pose.position.y=0;
    marker_array.markers[i].pose.position.z=0.075;
    marker_array.markers[i].pose.orientation.x=1.0;
    marker_array.markers[i].pose.orientation.y=0;
    marker_array.markers[i].pose.orientation.z=0;
    marker_array.markers[i].pose.orientation.w=1.0;
    marker_array.markers[i].color.r = 0.2f;
    marker_array.markers[i].color.g = 0.2f;
    marker_array.markers[i].color.b = 0.2f;
    marker_array.markers[i].color.a = 0.9f;
    /*
    i=6;
    marker_array.markers[i].header.frame_id = "/rs_link";
    marker_array.markers[i].header.stamp = ros::Time::now();
    marker_array.markers[i].ns = "cmd_vel_display";
    marker_array.markers[i].id = i;
    marker_array.markers[i].lifetime = ros::Duration();

    marker_array.markers[i].type = visualization_msgs::Marker::ARROW;
    marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array.markers[i].scale.x = 2.0;
    marker_array.markers[i].scale.y = 0.1;
    marker_array.markers[i].scale.z = 0.1;
    marker_array.markers[i].pose.position.x=0;
    marker_array.markers[i].pose.position.y=0;
    marker_array.markers[i].pose.position.z=0;
    marker_array.markers[i].pose.orientation.x=1.0;
    marker_array.markers[i].pose.orientation.y=0;
    marker_array.markers[i].pose.orientation.z=0;
    marker_array.markers[i].pose.orientation.w=1.0;
    marker_array.markers[i].color.r = 1.0f;
    marker_array.markers[i].color.g = 0.0f;
    marker_array.markers[i].color.b = 0.0f;
    marker_array.markers[i].color.a = 0.9f;
*/

    marker_pub.publish(marker_array);
}
