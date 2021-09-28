#pragma once

#ifndef STR(var)
    #define STR(var) #var
#endif

//addition status
enum class robot_status{
    run,
    stop,
    angleAdjust,
    safety_stop,
    recovery
};