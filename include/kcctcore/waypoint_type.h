/**
* @file waypoint_type.cpp
* @brief waypoint type define
* @author Shunya Hara
* @date 2021.3.6
* @details wp type name tag
*/

#pragma once
#include <string>

//addition status
enum class WaypointType{
    skip,
    normal,
    precision,
    person_detection

};

std::string WaypointType2String(WaypointType wpt){
    switch (wpt){
        case(WaypointType::skip):
            return "skip";
        case(WaypointType::normal):
            return "normal";
        case(WaypointType::precision):
            return "precision";
        case(WaypointType::person_detection):
            return "person_detection";
        default:
            return "";
    }
}