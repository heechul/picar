cmake_minimum_required(VERSION 2.8.3)
project(car_model)
find_package(catkin REQUIRED rospy roscpp std_msgs tf tf2 sensor_msgs geometry_msgs nav_msgs)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
catkin_install_python(PROGRAMS src/car_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
