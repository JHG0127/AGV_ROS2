# AGV_ROS2

ROS2 Galactic + Linux 20.04 focal


**Mapping**

1. ros2 launch myagv_bringup myagv.launch.py
2. ros2 launch myagv_cartographer cartographer.launch.py
3. ros2 run teleop_twist_keyboard teleop_twist_keyboard

4. ros2 run nav2_map_server map_saver_cli -f my_map

**Navigation**

1. ros2 launch myagv_bringup myagv.launch.py
2. ros2 launch myagv_navigation2 navigation2.launch.py

<parameter>
/src/myagv_navigation2/param.yaml
