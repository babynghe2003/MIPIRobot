from helper.MPU6050 import MPU_Init, read_raw_data, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H
from helper.StepperMotor import StepperMotor
import smbus					#import SMBus module of I2C
from time import sleep


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address
sleep(1)
MPU_Init()

leftmotor = StepperMotor(dir_pin=7, step_pin=8)
rightmotor = StepperMotor(dir_pin=21, step_pin=20)

leftmotor.RightDirec = False

P = 0
I = 0
D = 0

Kp = 5
Ki = 3
Kd = 10

leftmotor.start()
rightmotor.start()


while True:
    acc_x = read_raw_data(ACCEL_XOUT_H)
    gyro_x = read_raw_data(GYRO_XOUT_H)
    Ax = acc_x/16384.0*90.0
    Gx = gyro_x/131.0

    P = Kp * Ax
    I += Ki * Ax
    D = Gx

    PID = P + I + D
    
    leftmotor.set(PID)
    rightmotor.set(PID)




