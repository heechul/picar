cmake_minimum_required(VERSION 2.8.3)
project(car_model)
find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs tf sensor_msgs geometry_msgs nav_msgs)
catkin_package()

## Uncomment if the package has a setup.py
catkin_python_setup()

catkin_install_python(PROGRAMS src/car_publisher.py src/twist_driver.py src/car_navigator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})
