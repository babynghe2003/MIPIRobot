import RPi.GPIO as GPIO
from time import sleep
from threading import Thread, Lock


class StepperMotor:
    def __init__(self, dir_pin = 7, step_pin = 8, max_limit = 0.007, min_limit = 0.0015) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.DIR = dir_pin
        self.STEP = step_pin
        self.RightDirec = True
        self.end_speed = 0.007
        self.MAX = max_limit 
        self.MIN = min_limit
        self.lock_thread = Lock()

    def set(self, speed):
        if (speed > 0):
            if (self.RightDirec):
                GPIO.output(self.DIR, GPIO.HIGH)
            else:
                GPIO.output(self.DIR, GPIO.LOW)
        else:
            if (self.RightDirec):
                GPIO.output(self.DIR, GPIO.LOW)
            else:
                GPIO.output(self.DIR, GPIO.HIGH)
        speed = abs(speed)
        if (speed > 100):
            speed = 100 
        end_speed = self.MAX - speed / 100 * (self.MAX - self.MIN) 

    def run_motor(self):
        while True:
            if self.end_speed > self.MAX:
                continue
            GPIO.output(self.STEP, GPIO.HIGH)
            sleep(self.end_speed)
            GPIO.output(self.STEP, GPIO.LOW)
            sleep(self.end_speed)

    def start(self):
        start_thread = Thread(target=self.run_motor)
        start_thread.start()



# DIR = 7
# STEP = 8
# GPIO.setup(DIR, GPIO.OUT)
# GPIO.setup(STEP, GPIO.OUT)

# DIR2 = 21
# STEP2 = 20
# GPIO.setup(DIR2, GPIO.OUT)
# GPIO.setup(STEP2, GPIO.OUT)


# while  True:
    # GPIO.output(DIR, GPIO.HIGH)
    # GPIO.output(DIR2, GPIO.LOW)
    # sleep(0.1)
    # for i in range(0,100):
            # GPIO.output(STEP2, GPIO.HIGH)
            # GPIO.output(STEP, GPIO.HIGH)
            # sleep(0.002)
            # GPIO.output(STEP2, GPIO.LOW)
            # GPIO.output(STEP, GPIO.LOW)
            # sleep(0.002)
    # GPIO.output(DIR, GPIO.LOW)
    # GPIO.output(DIR2, GPIO.HIGH)
    # sleep(0.1)
    # for i in range(0,100):
            # GPIO.output(STEP2, GPIO.HIGH)
            # GPIO.output(STEP, GPIO.HIGH)
            # sleep(0.005)
            # GPIO.output(STEP2, GPIO.LOW)
            # GPIO.output(STEP, GPIO.LOW)
            # sleep(0.005)
