#include "mpu_6050.h"
#include "Arduino.h"

MPU6050::MPU6050(TwoWire &w){
  wire = &w;

}

void MPU6050::begin(float alpha){
  //MPU6050 Initializing & Reset
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x6B);  // PWR_MGMT_1 register
  wire->write(0);     // set to zero (wakes up the MPU-6050)
  wire->endTransmission(true);

  //MPU6050 Clock Type
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x6B);  // PWR_MGMT_1 register
  wire->write(0x03);     // Selection Clock 'PLL with Z axis gyroscope reference'
  wire->endTransmission(true);

  //MPU6050 Gyroscope Configuration Setting
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x1B);  // Gyroscope Configuration register
  //wire->write(0x00);     // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
  //wire->write(0x08);     // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
  //wire->write(0x10);     // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
  wire->write(0x18);     // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]
  wire->endTransmission(true);

  //MPU6050 Accelerometer Configuration Setting
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x1C);  // Accelerometer Configuration register
  //wire->write(0x00);     // AFS_SEL=0, Full Scale Range = +/- 2 [g]
  //wire->write(0x08);     // AFS_SEL=1, Full Scale Range = +/- 4 [g]
  wire->write(0x10);     // AFS_SEL=2, Full Scale Range = +/- 8 [g]
  //wire->write(0x18);     // AFS_SEL=3, Full Scale Range = +/- 10 [g]
  wire->endTransmission(true);

  //MPU6050 DLPF(Digital Low Pass Filter)
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x1A);  // DLPF_CFG register
  wire->write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 
  //wire->write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz 
  //wire->write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz 
  //wire->write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz 
  //wire->write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz 
  //wire->write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz 
  //wire->write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz 
  wire->endTransmission(true);
  this->alpha = alpha;
}


void MPU6050::update(){
  // Read raw data of MPU6050
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  wire->endTransmission(false);
  wire->requestFrom(MPU6050_ADDR,14);  // request a total of 14 registers
  AcX=wire->read()<<8|wire->read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=wire->read()<<8|wire->read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=wire->read()<<8|wire->read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=wire->read()<<8|wire->read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=wire->read()<<8|wire->read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=wire->read()<<8|wire->read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=wire->read()<<8|wire->read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)


 // Convert accelerometer to gravity value
  GAcX = (float) AcX / 4096.0;
  GAcY = (float) AcY / 4096.0;
  GAcZ = (float) AcZ / 4096.0;


  acc_pitch = atan ((GAcY - (float)MPU6050_AYOFFSET/4096.0) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951; // 180 / PI = 57.29577951
  acc_roll = - atan ((GAcX - (float)MPU6050_AXOFFSET/4096.0) / sqrt(GAcY * GAcY + GAcZ * GAcZ)) * 57.29577951; 
  //acc_yaw = atan ((GAcZ - (float)MPU6050_AZOFFSET/4096.0) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951;
  acc_yaw = atan (sqrt(GAcX * GAcX + GAcZ * GAcZ) / (GAcZ - (float)MPU6050_AZOFFSET/4096.0)) * 57.29577951; 

  // Calculate Pitch, Roll & Yaw from Gyroscope value reflected cumulative time factor
  Cal_GyX += (float)(GyX - MPU6050_GXOFFSET) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
  Cal_GyY += (float)(GyY - MPU6050_GYOFFSET) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
  Cal_GyZ += (float)(GyZ - MPU6050_GZOFFSET) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)

  // Calculate Pitch, Roll & Yaw by Complementary Filter
  // Reference is http://www.geekmomprojects.com/gyroscopes-and-accelerometers-on-a-chip/
  // Filtered Angle = α × (Gyroscope Angle) + (1 − α) × (Accelerometer Angle)     
  // where α = τ/(τ + Δt)   and   (Gyroscope Angle) = (Last Measured Filtered Angle) + ω×Δt
  // Δt = sampling rate, τ = time constant greater than timescale of typical accelerometer noise
  angle_pitch = alpha * (((float)(GyX - MPU6050_GXOFFSET) * 0.000244140625) + angle_pitch) + (1 - alpha) * acc_pitch;
  angle_roll = alpha * (((float)(GyY - MPU6050_GYOFFSET) * 0.000244140625) + angle_roll) + (1 - alpha) * acc_roll;
  angle_yaw += (float)(GyZ - MPU6050_GZOFFSET) * 0.000244140625; // Accelerometer doesn't have yaw value


}
