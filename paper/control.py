
while True:
  # 1. read from the forward camera
  frame = camera.read()
  # 2. convert to 200x66 rgb pixels
  frame = preprocess(frame)
  # 3. perform inferencing operation
  angle = DNN_inferencing(frame)
  # 4. motor control                       
  steering_motor_control(angle)
  # 5. wait till next period begins                       
  wait_till_next_period() 
  
