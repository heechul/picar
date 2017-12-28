# servo config
#   steering:
#     right: 952 us, center: 1516 us, left: 2152 us
#   throttle:
#     rew: 876,   stop: 1476,  ffw: 2070
str_min_pwm =  952
str_max_pwm = 2152

thr_max_pwm = 2070
thr_neu_pwm = 1476

thr_cap_pct = 0.16  # 20% max

# control loop
period = 0.05 # sec (=50ms)

# camera
cam_width=320
cam_height=240
cam_fps=20 # 50ms/period

# data output file names
data_video='out-video.avi'
data_steer='out-key.csv'
