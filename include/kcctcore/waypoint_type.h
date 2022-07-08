/**
* @file waypoint_type.cpp
* @brief waypoint type define
* @author Shunya Hara -> Akiro Harada
* @date 2021.3.6 -> 2022.7.4
* @details wp type name tag
*/

#pragma once
#include <string>

enum class WaypointType{
    skip,
    normal,
    precision,
    person_detection,
    recursion_start,
    recursion_end
};

std::unordered_map<WaypointType, std::string> uom_waypointType2String{
    {WaypointType::skip, "skip"},
    {WaypointType::normal, "normal"},
    {WaypointType::precision, "precision"},
    {WaypointType::person_detection, "person_detection"},
    {WaypointType::recursion_start, "recursion_start"},
    {WaypointType::recursion_end, "recursion_end"}
    };

std::unordered_map<std::string, WaypointType> uom_string2WaypointType{
    {"skip", WaypointType::skip},
    {"normal", WaypointType::normal},
    {"precision", WaypointType::precision},
    {"person_detection", WaypointType::person_detection},
    {"recursion_start", WaypointType::recursion_start},
    {"recursion_end", WaypointType::recursion_end}
    };

std::string WaypointType2String(WaypointType wpType){
    return uom_waypointType2String[wpType];
}

WaypointType String2WaypointType(std::string str){
    return uom_string2WaypointType[str];
}