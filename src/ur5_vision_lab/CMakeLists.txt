cmake_minimum_required(VERSION 3.0.2)
project(ur5_vision_lab)
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs geometry_msgs
  visualization_msgs std_msgs gazebo_msgs cv_bridge message_filters
  tf2_ros tf2_geometry_msgs)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/spawn_scene.py
  scripts/camera_listener.py
  scripts/perception_node.py
  scripts/localization_node.py
  scripts/move_cube.py
  scripts/status_node.py
  scripts/status_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config models rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
