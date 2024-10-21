import sys
sys.path.append('/usr/local/lib/python3.6/dist-packages/adafruit_servokit.py')
from adafruit_servokit import ServoKit
import time
from datetime import datetime, timedelta


#try:
#    import adafruit_servokit
#except Exception as e:
#    pass


duration = 10
v = 0.5
motor_channel = 0

kit = ServoKit(channels=16)

kit.continuous_servo[motor_channel].throttle = v

# end_time = datetime.now() + timedelta(seconds = duration)
# while datetime.now() < end_time:
#     kit.continuous_servo[motor_channel].throttle = v
#     kit.continuous_servo[motor_channel].throttle = -v
