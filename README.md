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


**parameter**

/src/myagv_navigation2/param.yaml


**Fin**

1. ros2 launch myagv_bringup myagv.launch.py

2. ros2 launch myagv_navigation2 navigation2.launch.py

3. ros2 run agv_nav patrol_and_detect

4. ros2 run agv_nav yolo

5. Yolov8 ROS2 노드를 켜주고 close 토픽을 받으면 delivery & patrol.
