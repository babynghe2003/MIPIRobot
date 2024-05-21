#ifndef MPU6050_TOCKN_H
#define MPU6050_TOCKN_H

#include "Arduino.h"
#include "Wire.h"

#define MPU6050_ADDR 0x68
#define MPU6050_AXOFFSET 158
#define MPU6050_AYOFFSET 9
#define MPU6050_AZOFFSET -91
#define MPU6050_GXOFFSET 19
#define MPU6050_GYOFFSET -42
#define MPU6050_GZOFFSET -26

class MPU6050
{
public:
    MPU6050(TwoWire &w);

    void begin(float alpha = 0.96);

    int16_t getRawAccX() { return AcX; };
    int16_t getRawAccY() { return AcX; };
    int16_t getRawAccZ() { return AcZ; };

    int16_t getRawTemp() { return Tmp; };

    int16_t getRawGyroX() { return Tmp; };
    int16_t getRawGyroY() { return GyY; };
    int16_t getRawGyroZ() { return GyZ; };

    void update();

    float getAnglePitch() { return angle_pitch; };
    float getAngleRoll() { return angle_roll; };
    float getAngleYaw() { return angle_yaw; };

private:
    TwoWire *wire;

    int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // Raw data of MPU6050
    float GAcX, GAcY, GAcZ;                    // Convert accelerometer to gravity value
    float Cal_GyX, Cal_GyY, Cal_GyZ;           // Pitch, Roll & Yaw of Gyroscope applied time factor
    float acc_pitch, acc_roll, acc_yaw;        // Pitch, Roll & Yaw from Accelerometer
    float angle_pitch, angle_roll, angle_yaw;  // Angle of Pitch, Roll, & Yaw
    float alpha;                       // Complementary constant
};

#endif
