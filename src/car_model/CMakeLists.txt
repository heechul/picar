cmake_minimum_required(VERSION 2.8.3)
project(car_model)
find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs tf sensor_msgs geometry_msgs nav_msgs)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_definitions(-std=c++11)
add_executable(car_publisher src/car_publisher.cpp)
target_link_libraries(car_publisher ${catkin_LIBRARIES})
