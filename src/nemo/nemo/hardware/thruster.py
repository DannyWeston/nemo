import time
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

class Thruster:
    def __init__(self, gpio_pin, min_duty=1, max_duty=7):
        self.pin = gpio_pin

        self.min_duty = min_duty
        self.max_duty = max_duty

        self.centre_duty = (max_duty - min_duty) / 2.0

        self.value = 0.0

        self.handle = self.initialise(gpio_pin)

    def initialise(self, gpio_pin):
        GPIO.setup(self.pin, GPIO.OUT)

        handle = GPIO.PWM(gpio_pin, 50)
        handle.start(0)

        return handle

    def set_speed(self, speed):
        frac = speed / 100.0
        self.value = self.centre_duty + ((self.max_duty - self.centre_duty) * frac)

        self.ChangeDutyCycle(self.value)

    def get_speed(self):
        return (100.0 * (self.value - self.centre_duty)) / (self.max_duty - self.centre_duty) 