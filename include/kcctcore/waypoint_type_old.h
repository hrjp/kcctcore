/**
* @file waypoint_type.cpp
* @brief waypoint type define
* @author Shunya Hara
* @date 2021.3.6
* @details wp type name tag
*/
#pragma once

enum waypoint_type{
    waypoint_type_free,//0
    waypoint_type_lidar,//1
    waypoint_type_stop,//2
    waypoint_type_skip,//3
    waypoint_type_camera//4

};