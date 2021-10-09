#!/bin/bash

roslaunch kcctcore bag.launch &

sleep 1;rosbag play /home/user/rosbag/2021-10-07-14-15-16.bag --clock --topic /velodyne_points /float_sensor_data /initialpose &

sleep 1;roslaunch kcctcore mcl_3dl.launch
