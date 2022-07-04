/**
* @file button_status.cpp
* @brief rviz pugin button status
* @author Shunya Hara
* @date 2021.3.6
* @details button name tag
*/
#pragma once
#include <string>
#include <unordered_map>

enum class ButtonState{
    FREE,
    START,
    PAUSE ,
    INITIAL_POSE,
    PLUS,
    MYNUS,
    RESET,
    Z_PUBLISH
};

std::unordered_map<ButtonState, std::string> uom_buttonState2String{
    {ButtonState::FREE, "free"},
    {ButtonState::START, "start"},
    {ButtonState::PAUSE, "pause"},
    {ButtonState::INITIAL_POSE, "initial_pose"},
    {ButtonState::PLUS, "plus"},
    {ButtonState::MYNUS, "mynus"},
    {ButtonState::RESET, "reset"},
    {ButtonState::Z_PUBLISH, "z_publish"}
    };

std::unordered_map<std::string, ButtonState> uom_string2ButtonState{
    {"free", ButtonState::FREE},
    {"start", ButtonState::START},
    {"pause", ButtonState::PAUSE},
    {"initial_pose", ButtonState::INITIAL_POSE},
    {"plus", ButtonState::PLUS},
    {"mynus", ButtonState::MYNUS},
    {"reset", ButtonState::RESET},
    {"z_publish", ButtonState::Z_PUBLISH}
    };

std::string ButtonState2String(ButtonState bs){
    return uom_buttonState2String[bs];
}

ButtonState String2ButtonState(std::string str){
    return uom_string2ButtonState[str];
}