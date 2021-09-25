# kcctcore
## navigation_master
ナビゲーションのマスター司令ノード
### publisher
* selected_cmd_vel [geometry_msgs::Twist]
### subscriber
* mcl_cmd_vel [geometry_msgs::Twist]
* camera_cmd_vel [geometry_msgs::Twist]
> for person track
* buttons [std_msgs::Int32]
* waypoint/now [std_msgs::Int32]
* waypoint/type [std_msgs::Int32MultiArray]
> robot mode type


## teensy_handler
teensy（マイコン）との橋渡し
### publisher
* robot_linear_vel [geometry_msgs::Twist]
* odom [nav_msgs::Odometry]
* imu/data [sensor_msgs::Imu]
* imu/mag [sensor_msgs::MagneticField]
### subscriber
* float_sensor_data [Float32MultiArray]
> various data of robot from teensy