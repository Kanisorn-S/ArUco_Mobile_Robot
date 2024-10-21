from motor import stop
import Adafruit_PCA9685

# Initialize PCA
servo = Adafruit_PCA9685.PCA9685(address = 0x40, busnum = 1)
servo.set_pwm_freq(60)

# Set motor channel
motor_channel_left = 0
motor_channel_right = 1
motor_unload = 2

stop(servo, motor_channel_left, motor_channel_right)


