#include "mpu_6050.h"
#include "Arduino.h"

MPU6050::MPU6050(TwoWire &w){
  wire = &w;

}

void MPU6050::begin(float alpha){
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x6B); 
  wire->write(0);    
  wire->endTransmission(true);


  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x6B); 
  wire->write(0x03);    
  wire->endTransmission(true);


  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x1B); 
  wire->write(0x18);    
  wire->endTransmission(true);


  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x1C);  
  wire->write(0x10);   
  wire->endTransmission(true);


  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x1A); 
  wire->write(0x00);     
  wire->endTransmission(true);
  this->alpha = alpha;
}


void MPU6050::update(){

  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x3B); 
  wire->endTransmission(false);
  wire->requestFrom(MPU6050_ADDR,14); 
  AcX=wire->read()<<8|wire->read(); 
  AcY=wire->read()<<8|wire->read(); 
  AcZ=wire->read()<<8|wire->read(); 
  Tmp=wire->read()<<8|wire->read(); 
  GyX=wire->read()<<8|wire->read(); 
  GyY=wire->read()<<8|wire->read(); 
  GyZ=wire->read()<<8|wire->read(); 



  GAcX = (float) AcX / 4096.0;
  GAcY = (float) AcY / 4096.0;
  GAcZ = (float) AcZ / 4096.0;


  acc_pitch = atan ((GAcY - (float)MPU6050_AYOFFSET/4096.0) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951;
  acc_roll = - atan ((GAcX - (float)MPU6050_AXOFFSET/4096.0) / sqrt(GAcY * GAcY + GAcZ * GAcZ)) * 57.29577951; 

  acc_yaw = atan (sqrt(GAcX * GAcX + GAcZ * GAcZ) / (GAcZ - (float)MPU6050_AZOFFSET/4096.0)) * 57.29577951; 


  Cal_GyX += (float)(GyX - MPU6050_GXOFFSET) * 0.000244140625;
  Cal_GyY += (float)(GyY - MPU6050_GYOFFSET) * 0.000244140625;
  Cal_GyZ += (float)(GyZ - MPU6050_GZOFFSET) * 0.000244140625;

  angle_pitch = alpha * (((float)(GyX - MPU6050_GXOFFSET) * 0.000244140625) + angle_pitch) + (1 - alpha) * acc_pitch;
  angle_roll = alpha * (((float)(GyY - MPU6050_GYOFFSET) * 0.000244140625) + angle_roll) + (1 - alpha) * acc_roll;
  angle_yaw += (float)(GyZ - MPU6050_GZOFFSET) * 0.000244140625;


}
