/**
* @file button_status.cpp
* @brief rviz pugin button status
* @author Shunya Hara
* @date 2021.3.6
* @details button name tag
*/
#pragma once

enum buttons_status{
    buttons_status_free,//0
    buttons_status_start,//1
    buttons_status_pause,//2
    buttons_status_initialpose,//3
    buttons_status_plus,//4
    buttons_status_mynus,//5
    buttons_status_reset,//6
    buttons_status_zpublish//7
};