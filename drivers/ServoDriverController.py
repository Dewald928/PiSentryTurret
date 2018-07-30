# Import the PCA9685 module.
from drivers import Adafruit_PCA9685

#===========================================================================================
# This class translates the position of the servo to the
# corresponding PWM signal.
#
# This code comes from the example code of the Adafruit PCA9685 library
# https://github.com/adafruit/Adafruit_Python_PCA9685
#
# move() is called with the servo positions in range -1(left/down)... 0(center)...1(right,up)
#===========================================================================================

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)


class ServoDriver :

    def __init__(self):
        # PWM setting
        FREQ = 60 # frequency set to 60 Hz
        self.pwm = Adafruit_PCA9685.PCA9685()
        # Alternatively specify a different address and/or bus:
        # pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)
        self.pwm.set_pwm_freq(FREQ)
        pulse_length = 1000000              #1,000,000 us per second
        pulse_length //= FREQ               #60 Hz (us per period)
        pulse_length //= 4096               #12 bit of resolution
        self.pulse_factor = 1000 / pulse_length
        #Servo range (test your servos to see milisecond range)
        self.pulse_center = 1.6
        self.pulse_range = 1.1

    def move(self, servo, position):
        if -1 < position < 1: #check if in range -1...0...1
            pulse = self.pulse_center + (position*self.pulse_range) #standard us pulse on servo spec in milisecond
            pulse *= self.pulse_factor      #convert to 12 resolution logic #120-600
            self.pwm.set_pwm(servo,0,int(pulse))






