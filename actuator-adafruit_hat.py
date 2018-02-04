from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

# init
mh = Adafruit_MotorHAT(addr=0x60)

def init():
    global steering, throttle
    steering = mh.getMotor(1)
    throttle = mh.getMotor(4)
    
# throttle
cur_speed = 0
MAX_SPEED = 255

def set_speed(speed):
    global cur_speed
    speed = int(MAX_SPEED * speed / 100)
    cur_speed = min(MAX_SPEED, speed)
    print ("speed: %d" % cur_speed)

def get_speed():
    global cur_speed    
    return int(cur_speed * 100 / MAX_SPEED)

def stop():
    throttle.setSpeed(0)
    steering.setSpeed(0)
    throttle.run(Adafruit_MotorHAT.RELEASE)
    steering.run(Adafruit_MotorHAT.RELEASE)
    
def ffw():
    global cur_speed        
    throttle.setSpeed(cur_speed)    
    throttle.run(Adafruit_MotorHAT.FORWARD)    

def rew():
    global cur_speed        
    throttle.setSpeed(cur_speed)    
    throttle.run(Adafruit_MotorHAT.BACKWARD)

# steering
def center():
    steering.setSpeed(0)
def left():
    steering.setSpeed(MAX_SPEED)
    steering.run(Adafruit_MotorHAT.BACKWARD)        
def right():
    steering.setSpeed(MAX_SPEED)
    steering.run(Adafruit_MotorHAT.FORWARD)


# exit    
def turn_off():
    stop()
    center()
