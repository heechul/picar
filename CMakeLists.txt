cmake_minimum_required(VERSION 2.8.3)
project(picar_base)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs)
catkin_package()

## Uncomment if the package has a setup.py
catkin_python_setup()

catkin_install_python(PROGRAMS deeptesla-directory/controller.py
  deeptesla-directory/rviz_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})
