roscore &
sleep 5
roslaunch rtimulib_ros rtimulib_ros.launch &  # publish 'imu' channel
rosrun teleop_twist_joy teleop_node &         # publish 'cmd_vel' output from joystick input
./servod &              # enable pwm output for the speed motor
./src/motor_driver.py & # subscribe 'cmd_vel' channel
# rosrun teleop_twist_keyboard teleop_twist_keyboard.py &
