#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <fastStepper.h>
#include <mpu_6050.h>
#include <BluetoothSerial.h>

#define LEN 19
#define LDIR 5
#define LSTEP 18
#define REN 4
#define RDIR 15
#define RSTEP 2
#define BTN_STOP_ALARM    0

#define LSERVO 14
#define RSERVO 12

#define reverseLeftMotor false
#define reverseRightMotor false

#define MAX_ACC 10

void IRAM_ATTR motLeftTimerFunction();
void IRAM_ATTR motRightTimerFunction();
void caculate_pid_left();
void caculate_pid_right();
float mymap(float, float, float, float, float);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
MPU6050 mpu6050(Wire);

fastStepper motLeft(LSTEP, LDIR, 0, true, motLeftTimerFunction);
fastStepper motRight(RSTEP, RDIR, 1, false, motRightTimerFunction);

bool mode = true;

Servo myservo = Servo();

long sampling_timer;  

// at 0* 11.56 1. 15.00        -0.00100000 -0.00001222 0.02233333
float Kp = 11.5, Ki = 1.2, Kd = 20; // at 30*: 11.5 10 20
float Km = -0.00295, Kc = 0.029, Kt = -0.000024; // at 30* Km = -0.00355, Kc = 0.029, Kt = -0.000024;
float Pl, Il, Dl, Ml, Cl, PIDl, MCl, Tl;
float Pr, Ir, Dr, Mr, Cr, PIDr, MCr, Tr;
float error = 0, lastError = 0, lastLastError=0;

int leftAngle = 30, rightAngle = 30;
long v_timer;
float VL = 0, VR = 0;

int microStep = 16;

float offsetAngle = 2;
float useOffsetAngle = 2;

bool isForward = false, isBackward = false;
long target_step_left = 0;
long target_step_right = 0;

String message = "";

bool running = false;

void IRAM_ATTR motLeftTimerFunction() {
  portENTER_CRITICAL_ISR(&motLeft.timerMux);
  motLeft.timerFunction();
  portEXIT_CRITICAL_ISR(&motLeft.timerMux);
}
void IRAM_ATTR motRightTimerFunction() {
  portENTER_CRITICAL_ISR(&motRight.timerMux);
  motRight.timerFunction();
  portEXIT_CRITICAL_ISR(&motRight.timerMux);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("MipiRobot");

  motLeft.init();
  motRight.init();
  motLeft.microStep = microStep;
  motRight.microStep = microStep;

  myservo.write(LSERVO, leftAngle);
  myservo.write(RSERVO, 180-rightAngle);

  motLeft.speed = 0;
  motRight.speed = 0;
  motLeft.update();
  motRight.update();
  

  Wire.begin();
  mpu6050.begin(0.996);
  // init_MPU6050();

}

void loop() {
  if (SerialBT.available()) {
    message = SerialBT.readStringUntil('\n'); 
    Serial.println(message);  
    if (message[0] == 'J'){
      if (mode)
        Km = mymap(message.substring(1).toInt(), 0, 180, -0.01, 0.01); // 0.0041
      else {
        Kp = mymap(message.substring(1).toInt(), 0, 180, -15, 15); // 0.0041

        // offsetAngle = mymap(message.substring(1).toInt(), 0, 180, -5, 5);
        // useOffsetAngle = offsetAngle;
      }
      
    } else if (message[0] == 'K'){
      // if (mode)
      //   Kc = mymap(message.substring(1).toInt(), 0, 180, -0.03, 0.03);
      // else {
      //   Kp = mymap(message.substring(1).toInt(), 0, 180, -20, 20);

        rightAngle = mymap(message.substring(1).toInt(), 0, 180, 0, 30);
        myservo.write(RSERVO, 180 - rightAngle);
        leftAngle = mymap(message.substring(1).toInt(), 0, 180, 0, 30);
        myservo.write(LSERVO, leftAngle);
        offsetAngle = mymap((leftAngle + rightAngle)/2, 0, 30, -1.2, 1.8);
        Kp = mymap((leftAngle + rightAngle)/2, 0, 30, 11.5, 12);
        Ki = mymap((leftAngle + rightAngle)/2, 0, 30, 1, 1.2);
        Km = mymap((leftAngle + rightAngle)/2, 0, 30, -0.00100000, -0.0035);
        Kc = mymap((leftAngle + rightAngle)/2, 0, 30, 0.02233333, 0.026);
        Kt = mymap((leftAngle + rightAngle)/2, 0, 30, -0.00001222, -0.000024);
        // Ki = mymap((leftAngle + rightAngle)/2, 0, 30, 1, 1.2);
      // }
      
    } else if (message == "M"){
      running = true;
      Ir = 0;
      Il = 0;
    } else if (message == "m") {
      running = false;
    } else if (message == "X") {
      Ir = 0;
      Il = 0;
      Tl = 0;
      Tr = 0;
      target_step_left = 0;
      target_step_right = 0;
      motLeft.setStep(0);
      motRight.setStep(0);

    } else if (message == "L") {
      // target_step_left -=100;
      // target_step_right+=100;
      VL = -10;
      VR = 10;
    } else if (message == "R"){
      // target_step_left +=100;
      // target_step_right-=100;
      VL = 10;
      VR = -10;
    } else if (message == "S") {   
      VL = 0; VR = 0; 
      isForward = false;
      isBackward = false;
    } else if (message == "N") {
      mode = true;
    } else if (message == "n") {
      mode = false;
    } else if (message == "F"){
      isForward = true;
      target_step_left +=100;
      target_step_right +=100;
    } else if (message == "B") {
      isBackward = true;
      target_step_left -=100;
      target_step_right-=100;
    }
  }
  if (micros() - v_timer > 500){
    if (isForward){
      target_step_left += 10;
      target_step_right += 10;
    } else if (isBackward){
      target_step_left -=10;
      target_step_right-=10;
    } 
    v_timer = micros();
  }

  mpu6050.update();

  caculate_pid_left();
  caculate_pid_right();
  lastError = error;
  lastLastError = lastError;

    Serial.print(motLeft.getStep());
    Serial.print(' ');
    Serial.print(target_step_left);
    Serial.print('\t');
    Serial.print(Kp);
    Serial.print(' ');
    Serial.print(Ki);
    Serial.print(' ');
    Serial.print(Kd);
    Serial.print('\t');
    Serial.print(Km,8);
    Serial.print(' ');
    Serial.print(Kt,8);
    Serial.print(' ');
    Serial.print(Kc,8);
    Serial.print('\t');
    Serial.print(MCl);
    Serial.print(' ');
    Serial.print(PIDl);
    Serial.print(' ');
    Serial.print(useOffsetAngle);
    Serial.print(' ');
    Serial.print(mpu6050.getAnglePitch());
    Serial.print('\t');
    Serial.print(motLeft.getStep());
    Serial.print(' ');
    Serial.println(motRight.getStep());

  while(micros() - sampling_timer < 2000); //
  sampling_timer = micros(); //Reset the sampling timer  
}

void caculate_pid_left (){

  error = mpu6050.getAnglePitch() - useOffsetAngle;

  if (error > -30 && error < 30){
    Ml = Km*(target_step_left - motLeft.getStep());
    Cl = Kc*motLeft.speed;
    Tl += Kt*(target_step_left - motLeft.getStep());
    Tl = constrain(Tl, -10, 10);
    MCl = Ml + Cl+Tl;
    MCl = constrain(MCl, -10, 10);
    error += MCl;

    Pl = Kp * error;
    Il = constrain(Il + Ki * error, -500, 500);
    Dl = Kd*(((error - lastError)+(lastError - lastLastError))/2);
    PIDl = Pl + Il + Dl;
 
    PIDl = constrain(PIDl, -500, 500);
  } else {
    PIDl = 0;
  }
  if (running){
    motLeft.speed = constrain((PIDl + VL), motLeft.speed - MAX_ACC, motLeft.speed + MAX_ACC);
    motLeft.update();
  }else{
    motLeft.speed = 0;
    motLeft.update();
  }

}

void caculate_pid_right (){
  error = mpu6050.getAnglePitch() - useOffsetAngle;
  if (error > -30 && error < 30){

    Mr = Km*(target_step_right - motRight.getStep());
    Cr = Kc*motRight.speed;
    Tr += Kt*(target_step_right - motRight.getStep());
    Tr = constrain(Tl, -10, 10);
    MCr = Mr + Cr +Tl;
    MCr = constrain(MCr, -10, 10);
    error += MCr;

    Pr = Kp * error;
    Ir = constrain(Ir + Ki * error, -500, 500);
    Dr = Kd*Kd*(((error - lastError)+(lastError - lastLastError))/2);
    PIDr = Pr + Ir + Dr;
    
    PIDr = constrain(PIDl, -500, 500);
  } else {
    PIDl = 0;
  }
  if (running){
    motRight.speed = constrain((PIDr + VR), motRight.speed - MAX_ACC, motRight.speed + MAX_ACC);
    motRight.update();
  }else{
    motRight.speed = 0;
    motRight.update();
  }

}

float mymap(float x, float in_min, float in_max, float out_min, float out_max){
  if (in_min >= in_max){
    return 0;
  } 
  const float run = in_max - in_min;
  const float rise = out_max - out_min;
  const float delta = x - in_min;
  return (delta * rise) / run + out_min;
}