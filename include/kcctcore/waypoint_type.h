/**
* @file waypoint_type.cpp
* @brief waypoint type define
* @author Shunya Hara
* @date 2021.3.6
* @details wp type name tag
*/

#pragma once
#include <string>

enum class WaypointType{
    skip,
    normal,
    precision,
    person_detection
};

std::unordered_map<WaypointType, std::string> uom_waypointType2String{
    {WaypointType::skip, "skip"},
    {WaypointType::normal, "normal"},
    {WaypointType::precision, "precision"},
    {WaypointType::person_detection, "person_detection"}
    };

std::unordered_map<std::string, WaypointType> uom_string2WaypointType{
    {"skip", WaypointType::skip},
    {"normal", WaypointType::normal},
    {"precision", WaypointType::precision},
    {"person_detection", WaypointType::person_detection}
    };

std::string WaypointType2String(WaypointType wpType){
    return uom_waypointType2String[wpType];
}

WaypointType String2WaypointType(std::string str){
    return uom_string2WaypointType[str];
}