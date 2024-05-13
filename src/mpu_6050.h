#ifndef MPU6050_TOCKN_H
#define MPU6050_TOCKN_H

#include "Arduino.h"
#include "Wire.h"

#define MPU6050_ADDR 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1a
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_TEMP_H 0x41
#define MPU6050_TEMP_L 0x42

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
    MPU6050(TwoWire &w, float aC, float gC);

    void begin(float alpha = 0.96);

    void setGyroOffsets(float x, float y, float z);

    void writeMPU6050(byte reg, byte data);
    byte readMPU6050(byte reg);

    int16_t getRawAccX() { return rawAccX; };
    int16_t getRawAccY() { return rawAccY; };
    int16_t getRawAccZ() { return rawAccZ; };

    int16_t getRawTemp() { return rawTemp; };

    int16_t getRawGyroX() { return rawGyroX; };
    int16_t getRawGyroY() { return rawGyroY; };
    int16_t getRawGyroZ() { return rawGyroZ; };

    float getTemp() { return temp; };

    float getAccX() { return accX; };
    float getAccY() { return accY; };
    float getAccZ() { return accZ; };

    float getGyroX() { return gyroX; };
    float getGyroY() { return gyroY; };
    float getGyroZ() { return gyroZ; };

    void calcGyroOffsets(bool console = false, uint16_t delayBefore = 1000, uint16_t delayAfter = 3000);

    float getGyroXoffset() { return gyroXoffset; };
    float getGyroYoffset() { return gyroYoffset; };
    float getGyroZoffset() { return gyroZoffset; };

    void update();

    float getAccAngleX() { return angleAccX; };
    float getAccAngleY() { return angleAccY; };

    float getGyroAngleX() { return angleGyroX; };
    float getGyroAngleY() { return angleGyroY; };
    float getGyroAngleZ() { return angleGyroZ; };

    float getAngleX() { return angleX; };
    float getAngleY() { return angleY; };
    float getAngleZ() { return angleZ; };

    float getAnglePitch() { return angle_pitch; };
    float getAngleRoll() { return angle_roll; };
    float getAngleYaw() { return angle_yaw; };

private:
    TwoWire *wire;

    int16_t rawAccX, rawAccY, rawAccZ, rawTemp,
        rawGyroX, rawGyroY, rawGyroZ;

    float gyroXoffset, gyroYoffset, gyroZoffset;

    float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;

    float angleGyroX, angleGyroY, angleGyroZ,
        angleAccX, angleAccY, angleAccZ;

    float angleX, angleY, angleZ;

    float interval;
    long preInterval;

    float accCoef, gyroCoef;

    int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // Raw data of MPU6050
    float GAcX, GAcY, GAcZ;                    // Convert accelerometer to gravity value
    float Cal_GyX, Cal_GyY, Cal_GyZ;           // Pitch, Roll & Yaw of Gyroscope applied time factor
    float acc_pitch, acc_roll, acc_yaw;        // Pitch, Roll & Yaw from Accelerometer
    float angle_pitch, angle_roll, angle_yaw;  // Angle of Pitch, Roll, & Yaw
    float alpha;                       // Complementary constant
};

#endif
