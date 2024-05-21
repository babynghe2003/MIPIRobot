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

void IRAM_ATTR motLeftTimerFunction();
void IRAM_ATTR motRightTimerFunction();
void caculate_pid_left();
void caculate_pid_right();
float mymap(float, float, float, float, float);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
MPU6050 mpu6050(Wire);

fastStepper motLeft(LSTEP, LDIR, 0, true, motLeftTimerFunction);
fastStepper motRight(RSTEP, RDIR, 1, false, motRightTimerFunction);

bool mode = true;

Servo myservo = Servo();

long sampling_timer;  

float Kp = 7., Ki = 1.2, Kd = 12; // at 30*: 3. 0.48 10
float Km = -0.0045, Kc = 0.02;
float Pl, Il, Dl, Ml, Cl, PIDl, MCl;
float Pr, Ir, Dr, Mr, Cr, PIDr, MCr;
float error = 0, lastError = 0;

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
  portENTER_CRITICAL_ISR(&timerMux);
  motLeft.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}
void IRAM_ATTR motRightTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motRight.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
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
        Kp = mymap(message.substring(1).toInt(), 0, 180, -10, 10); // 0.0041

        // offsetAngle = mymap(message.substring(1).toInt(), 0, 180, -5, 5);
        // useOffsetAngle = offsetAngle;
      }
      
    } else if (message[0] == 'K'){
      if (mode)
        Kc = mymap(message.substring(1).toInt(), 0, 180, -0.02, 0.02);
      else {
        Kd = mymap(message.substring(1).toInt(), 0, 180, -20, 20);

        // rightAngle = mymap(message.substring(1).toInt(), 0, 180, 0, 30);
        // myservo.write(RSERVO, 180 - rightAngle);
        // leftAngle = mymap(message.substring(1).toInt(), 0, 180, 0, 30);
        // myservo.write(LSERVO, leftAngle);
        // offsetAngle = mymap((leftAngle + rightAngle)/2, 0, 30, -1.2, 1.8);
        // Ki = mymap((leftAngle + rightAngle)/2, 0, 30, 1, 0.5);
      }
      
    } else if (message == "M"){
      running = true;
      Ir = 0;
      Il = 0;
    } else if (message == "m") {
      running = false;
    } else if (message == "X") {
      Ir = 0;
      Il = 0;
      target_step_left = 0;
      target_step_right = 0;
      motLeft.setStep(0);
      motRight.setStep(0);

    } else if (message == "L") {
      VL = -10;
      VR = 10;
    } else if (message == "R"){
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
    } else if (message == "B") {
      isBackward = true;
    }
  }
  if (micros() - v_timer > 50000){
    if (isForward){
      useOffsetAngle = constrain(useOffsetAngle + 0.5, offsetAngle, offsetAngle + 2);
    } else if (isBackward){
      useOffsetAngle = constrain(useOffsetAngle - 0.5, offsetAngle - 2, offsetAngle);
    } else {
      if (useOffsetAngle > offsetAngle) {
        useOffsetAngle = constrain(useOffsetAngle - 0.5, offsetAngle, offsetAngle + 2);
      }
      else {
        useOffsetAngle = constrain(useOffsetAngle + 0.5, offsetAngle - 2, offsetAngle);
      }
    } 

    v_timer = micros();
  }

  mpu6050.update();

  caculate_pid_left();
  caculate_pid_right();
  lastError = error;

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
    Serial.print(Kc,8);
    Serial.print('\t');
    Serial.print(MCl);
    Serial.print(' ');
    Serial.print(PIDl);
    Serial.print(' ');
    Serial.print(useOffsetAngle);
    Serial.print(' ');
    Serial.println(mpu6050.getAnglePitch());

  while(micros() - sampling_timer < 2000); //
  sampling_timer = micros(); //Reset the sampling timer  
}

void caculate_pid_left (){

  error = mpu6050.getAnglePitch() - useOffsetAngle;

  if (error > -30 && error < 30){
    Ml = Km*(target_step_left - motLeft.getStep());
    Cl = Kc*motLeft.speed;
    MCl = Ml + Cl;
    MCl = constrain(MCl, -10, 10);
    error += MCl;

    Pl = Kp * error;
    Il = constrain(Il + Ki * error, -500, 500);
    Dl = Kd*(error - lastError);
    PIDl = Pl + Il + Dl;
 
    PIDl = constrain(PIDl, -500, 500);
  } else {
    PIDl = 0;
  }
  if (running){
    motLeft.speed = (PIDl + VL);
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
    MCr = Mr + Cr;
    MCr = constrain(MCr, -10, 10);
    error += MCr;

    Pr = Kp * error;
    Ir = constrain(Ir + Ki * error, -500, 500);
    Dr = Kd*(error - lastError);
    PIDr = Pr + Ir + Dr;
    
    PIDr = constrain(PIDl, -500, 500);
  } else {
    PIDl = 0;
  }
  if (running){
    motRight.speed = (PIDr + VR);
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