# LDS-006-Lidar-Sensor-Library

Simple ROS-publisher to attach sensor at my drone. Not used in any project.

## How to

Clone branch into your `catkin_ws/src` folder. Run `catkin build`. Source `catkin_ws/devel/setup.bash`. Then:

```
pip install -r requirements.txt
rosrun ros_noetic_lidar_sensor lds_generator.py
```

Optional arguments before starting `port` and `frame_id`:


```
rosparam set port /dev/serial0
rosparam set frame_id lidar_frame
```