#pragma once
#include <string>

//addition status
enum class waypoint_type{
    skip,
    normal,
    precision,
    person_detection

};

std::string waypoint_type_str(waypoint_type status){
    switch (status){
        case(waypoint_type::skip):
            return "skip";
        case(waypoint_type::normal):
            return "normal";
        case(waypoint_type::precision):
            return "precision";
        case(waypoint_type::person_detection):
            return "person_detection";
        default:
            return "";
    }
}