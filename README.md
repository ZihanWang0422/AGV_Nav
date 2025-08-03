这里包含了三个文件，其中底盘可以由 DDSM_ws 启动
```
cd ~/DDSM_ws
# 编译一下包
colcon build
#安装
source install/setup.bash
ros2 run robot_control serial_controller
```

其次是雷达，如果要编译记得先编译 driver
```
cd 2d_slam/src
colcon build --packages-select lslidar_driver 
colcon build
source install/setup.bash
ros2 launch lslidar_driver lsn10_launch.py
```
剩下的都在 `cd ~/fishbot_ws/src`,记得 source setup

map 到 odom 出错了，目前只跑 URDF，就开启baselink 到sensor之间的 TF
`ros2 launch fishbot_bringup fishbot_bringup.launch.py`

发布静态 tf 从map 到 odom     `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom`

最后开启

`ros2 launch fishbot_navigation2 navigation2.launch.py`