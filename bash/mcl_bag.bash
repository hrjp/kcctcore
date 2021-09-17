#!/bin/bash

roslaunch kcctcore bag.launch

rosbag play /home/user/rosbag/2021-09-15-17-34-47.bag --clock --topic /velodyne_points /float_sensor_data /initialpose


sleep 3s;
roslaunch kcctcore mcl_3dl.launch 
