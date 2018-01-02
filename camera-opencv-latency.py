# -*- coding: utf-8 -*-
"""
This utility measures the latency and frame rate for a 
USB webcam.  The preview window shows the timestamps to indicate latency
(you will need to point the camera at the monitor), and the console will
print out the frame per second. 
You may need to edit the camera channel, width, height, 
Created on Mon Feb 08 23:00:39 2016
@author: perrytsao 
More info at www.makehardware.com/webcam-latency
"""
import timeit
import time
import cv2

###############################################
width=320
height=240
fps=20
camera_channel=0
fname="camera_latency"
###############################################
wait_time=1

cv2.namedWindow("preview")
vc = cv2.VideoCapture(camera_channel)
vc.set(3, width)
vc.set(4, height)
vc.set(5, fps) 

################################################
time.sleep(2)
font = cv2.FONT_HERSHEY_SIMPLEX
try: 
    if vc.isOpened(): # try to get the first frame
        rval, frame_o = vc.read()
    else:
        rval = False
    ii=100
    toc=0    
    tic=timeit.default_timer()
    while rval:
        toc_old=toc
        toc=timeit.default_timer()        
        delta=toc-toc_old
        print("delta: %0.3f  fps: %0.3f" % (delta, 1/delta))
        cv2.putText(frame_o, "%0.3f" % (toc-tic), (50,200), font, 2, (255,255,255),4,0)       
        cv2.imshow("preview", frame_o)
        key = cv2.waitKey(wait_time) & 0xFF
      
        ## Monitor keyboard
        if key == ord('q'): # exit on ESC
            break
        elif key == ord('s'):
            cv2.imwrite(fname+str(ii)+".png", frame_o)
            ii+=1
        rval, frame_o = vc.read()
        
          
finally: 
    vc.release()
    cv2.destroyAllWindows()
