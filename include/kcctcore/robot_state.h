
/**
* @file robot_state.cpp
* @brief robot status
* @author Shunya Hara -> Akiro Harada
* @date 2021.3.6 -> 2022.7.4
* @details robot state tag
*/
#pragma once
#include <string>
#include <unordered_map>

enum class RobotState{
    RUN,
    STOP,
    ADJUST_ANGLE,
    SAFETY_STOP,
    RECOVERY,
    PERSON_TRACKING
};

std::unordered_map<RobotState, std::string> uom_robotState2String{
    {RobotState::RUN, "run"},
    {RobotState::STOP, "stop"},
    {RobotState::ADJUST_ANGLE, "adjust_angle"},
    {RobotState::SAFETY_STOP, "safety_stop"},
    {RobotState::RECOVERY, "recovery"},
    {RobotState::PERSON_TRACKING, "person_tracking"},
    };

std::unordered_map<std::string, RobotState> uom_string2RobotState{
    {"run", RobotState::RUN},
    {"stop", RobotState::STOP},
    {"adjust_angle", RobotState::ADJUST_ANGLE},
    {"safety_stop", RobotState::SAFETY_STOP},
    {"recovery", RobotState::RECOVERY},
    {"person_tracking", RobotState::PERSON_TRACKING}
    };

std::string RobotState2String(RobotState rs){
    return uom_robotState2String[rs];
}

RobotState String2RobotState(std::string str){
    return uom_string2RobotState[str];
}