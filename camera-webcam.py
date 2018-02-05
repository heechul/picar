import cv2
from threading import Thread,Lock
import time

enabled = False
cap = None
frame = None

# public API
# init(), read_frame(), stop()

def init(res=(320, 240), fps=30):
    print "Initilize camera."
    global cap, enabled,frame, cam_thr

    cap = cv2.VideoCapture(0)

    cap.set(3, res[0]) # width
    cap.set(4, res[1]) # height
    cap.set(5, fps)

    # start the camera thread
    enabled = True
    cam_thr = Thread(target=__update, args=())
    cam_thr.start()
    time.sleep(0.1)
    print ("camera init completed.")

def __update():
    global frame
    while enabled:
        ret, frame = cap.read() # blocking read.
    print ("Camera thread finished...")
    cap.release()        

def read_frame():
    return frame

def stop():
    global frame, enabled
    print ("Close the camera.")
    enabled = False
