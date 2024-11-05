#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <fastStepper.h>
#include <mpu_6050.h>
#include <WiFi.h>
#include <WebServer.h>

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

// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
MPU6050 mpu6050(Wire);
const char *ssid = "BalancingRobot";
const char *password = "12345678";

WebServer server(80);

fastStepper motLeft(LSTEP, LDIR, 0, true, motLeftTimerFunction);
fastStepper motRight(RSTEP, RDIR, 1, false, motRightTimerFunction);

bool mode = true;

Servo myservo = Servo();

long sampling_timer;  

// at 0* 11.56 1. 15.00        -0.00100000 -0.00001222 0.02233333
double Kp = 11.5, Ki = 1.2, Kd = 17; // at 30*: 11.5 10 20
double Km = -0.00245, Kc = 0.024, Kt = -0.000024; // at 30* Km = -0.00355, Kc = 0.029, Kt = -0.000024;
double Pl, Il, Dl, Ml, Cl, PIDl, MCl, Tl;
double Pr, Ir, Dr, Mr, Cr, PIDr, MCr, Tr;
double error = 0, lastError = 0, lastLastError=0;

double leftAngle = 28, rightAngle = 28, targetLeftAngle = 28, targetRightAngle = 28;
long v_timer;
double VL = 0, VR = 0;

int microStep = 16;

double offsetAngle = 2;
double useOffsetAngle = 2;

long targetLeftStep = 0;
long targetRightStep = 0;
long leftStep = 0;
long rightStep = 0;

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

String htmlPage() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";  // Thêm mã hóa UTF-8
  html += "<title>ESP32 Control</title>";
  html += "<style>";
  html += "body { display: flex; flex-direction: column; align-items: center; font-family: Arial, sans-serif; }";
  html += "h1 { margin-bottom: 20px; }";
  html += ".button-container { display: flex; justify-content: center; align-items: center; margin: 10px; }";
  html += "button { width: 100px; height: 100px; font-size: 16px; margin: 5px; }";
  html += ".slider-container { margin-top: 20px; width: 80%; max-width: 500px; }";
  html += ".slider-label { display: flex; justify-content: space-between;  }";
  html += ".slider{ width: 80%;  }";
  html += "</style></head><body>";
  
  // Tiêu đề
  html += "<h1>Điều khiển Robot</h1>";

  // Ba nút Start, Stop, Reset ở trên cùng
  html += "<div class='button-container'>";
  html += "<button onclick=\"location.href='/command?move=start'\">Start</button>";
  html += "<button onclick=\"location.href='/command?move=stop'\">Stop</button>";
  html += "<button onclick=\"location.href='/command?move=reset'\">Reset</button>";
  html += "</div>";
  
  // Bốn nút điều hướng Tiến, Trái, Phải, Lùi theo hình thoi
  html += "<div class='button-container'>";
  html += "<button onclick=\"location.href='/command?move=forward'\">Tiến</button>";
  html += "</div>";
  html += "<div class='button-container'>";
  html += "<button onclick=\"location.href='/command?move=left'\">Trái</button>";
  html += "<button onclick=\"location.href='/command?move=right'\">Phải</button>";
  html += "</div>";
  html += "<div class='button-container'>";
  html += "<button onclick=\"location.href='/command?move=backward'\">Lùi</button>";
  html += "</div>";
  
  // Hai nút Đứng lên và Ngồi xuống ở phía dưới
  html += "<div class='button-container'>";
  html += "<button onclick=\"location.href='/command?move=stand'\">Đứng lên</button>";
  html += "<button onclick=\"location.href='/command?move=sit'\">Ngồi xuống</button>";
  html += "</div>";

      // Sliders for Kp, Ki, Kd, Km, Kc, Kt
  html += "<div class='slider-container'>";
  String parameters[] = {"Kp", "Ki", "Kd", "Km", "Kc", "Kt"};

  int KpValue = mymap(Kp, 11.5 - 5, 11.5 + 5, 0, 100);
  int KiValue = mymap(Ki, 1.2 - 0.5, 1.2 + 0.5, 0, 100);
  int KdValue = mymap(Kd, 17-5, 17+5, 0, 100);
  int KmValue = mymap(Km, -0.00245 -0.001, -0.00245 +0.001, 0, 100);
  int KcValue = mymap(Kc, 0.024-0.01, 0.024+0.01, 0, 100);
  int KtValue = mymap(Kt, -0.000024 - 0.00001, -0.000024 + 0.00001, 0, 100);
  
  html += "<div class='slider-label'><label>Kp</label><span id='KpValue'>"+String(Kp)+"</span></div>";
  html += "<input type='range' min='1' max='100' value='"+String(KpValue)+"' class='slider'";
  html += "oninput=\"document.getElementById('KpValue').innerText=this.value;\" ";
  html += "onchange=\"location.href='/command?param=Kp&value=' + this.value\">";

  html += "<div class='slider-label'><label>Ki</label><span id='KpValue'>"+String(Ki)+"</span></div>";
  html += "<input type='range' min='1' max='100' value='"+String(KiValue)+"' class='slider'";
  html += "oninput=\"document.getElementById('KiValue').innerText=this.value;\" ";
  html += "onchange=\"location.href='/command?param=Ki&value=' + this.value\">";

  html += "<div class='slider-label'><label>Kd</label><span id='KdValue'>"+String(Kd)+"</span></div>";
  html += "<input type='range' min='1' max='100' value='"+String(KdValue)+"' class='slider'";
  html += "oninput=\"document.getElementById('KdValue').innerText=this.value;\" ";
  html += "onchange=\"location.href='/command?param=Kd&value=' + this.value\">";

  html += "<div class='slider-label'><label>Km</label><span id='KmValue'>"+String(Km, 8)+"</span></div>";
  html += "<input type='range' min='1' max='100' value='"+String(KmValue)+"' class='slider'";
  html += "oninput=\"document.getElementById('KmValue').innerText=this.value;\" ";
  html += "onchange=\"location.href='/command?param=Km&value=' + this.value\">";

  html += "<div class='slider-label'><label>Kc</label><span id='KcValue'>"+String(Kc, 8)+"</span></div>";
  html += "<input type='range' min='1' max='100' value='"+String(KcValue)+"' class='slider'";
  html += "oninput=\"document.getElementById('KcValue').innerText=this.value;\" ";
  html += "onchange=\"location.href='/command?param=Kc&value=' + this.value\">";

  html += "<div class='slider-label'><label>Kt</label><span id='KtValue'>"+String(Kt, 8)+"</span></div>";
  html += "<input type='range' min='1' max='100' value='"+String(KtValue)+"' class='slider'";
  html += "oninput=\"document.getElementById('KtValue').innerText=this.value;\" ";
  html += "onchange=\"location.href='/command?param=Kt&value=' + this.value\">";


  html += "</body></html>";
  return html;
}

void handleCommand() {
  if (server.hasArg("move")) {
    String command = server.arg("move");
    if (command == "forward") {
      targetLeftStep += 2000;
      targetRightStep += 2000; 
      Serial.println("Move Forward");
    } else if (command == "backward") {
      targetLeftStep -=2000;
      targetRightStep-=2000;
      Serial.println("Move Backward");
    } else if (command == "left") {
      targetLeftStep -=100;
      targetRightStep+=100;
      Serial.println("Move Left");
    } else if (command == "right") {
      targetLeftStep +=100;
      targetRightStep-=100;
      Serial.println("Move Right");
    } else if (command == "stand") {
      targetRightAngle = 10;
      targetLeftAngle = 10;


      Serial.println("Stand Up");
    } else if (command == "sit") {
      targetRightAngle = 30;
      myservo.write(RSERVO, 180 - rightAngle);
      targetLeftAngle = 30;
      myservo.write(LSERVO, leftAngle);
      offsetAngle = mymap((leftAngle + rightAngle)/2, 0, 30, -1.2, 1.8);
      Serial.println("Sit Down");
    } else if (command == "start"){
      running = true;
      Ir = 0;
      Il = 0;
    } else if (command == "stop"){
      running = false;
    } else if (command == "reset"){
      Ir = 0;
      Il = 0;
      Tl = 0;
      Tr = 0;
      targetLeftStep = 0;
      targetRightStep = 0;
      leftStep = 0;
      rightStep = 0;
      motLeft.setStep(0);
      motRight.setStep(0);
    }
  } else if (server.hasArg("param") && server.hasArg("value")) {
    String param = server.arg("param");
    int value = server.arg("value").toInt();  // Convert the value argument to an integer
//   float Kp = 11.5, Ki = 1.2, Kd = 20; // at 30*: 11.5 10 20
// float Km = -0.00295, Kc = 0.029, Kt = -0.000024; // at 30* Km = -0.00355, Kc = 0.029, Kt = -0.000024;
    if (param == "Kp") {
      Kp = mymap(value, 0, 100, 11.5 - 5, 11.5 + 5);
      Serial.println("Kp set to " + String(Kp));
    } else if (param == "Ki") {
      Ki = mymap(value, 0, 100, 1.2 - 0.5, 1.2 + 0.5);
      Serial.println("Ki set to " + String(Ki));
    } else if (param == "Kd") {
      Kd = mymap(value, 0, 100, 17-5, 17+5);
      Serial.println("Kd set to " + String(Kd));
    } else if (param == "Km") {
      Km = mymap(value, 0, 100, -0.00245 -0.001, -0.00245 +0.001);
      Serial.println("Km set to " + String(Km));
    } else if (param == "Kc") {
      Kc = mymap(value, 0, 100, 0.024-0.01, 0.024+0.01);
      Serial.println("Kc set to " + String(Kc));
    } else if (param == "Kt") {
      Kt = mymap(value, 0, 100, -0.000024 - 0.00001, -0.000024 + 0.00001);
      Serial.println("Kt set to " + String(Kt));
    }
  }
  server.send(200, "text/html", htmlPage());
}

void setup() {
  Serial.begin(115200);

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
  
    // Set up the ESP32 as an Access Point
  WiFi.softAP(ssid, password);
  IPAddress local_ip(192, 168, 4, 1);       // Default IP
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  
  Serial.println("ESP32 Access Point created");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  
  server.on("/", []() { server.send(200, "text/html", htmlPage()); });
  server.on("/command", handleCommand);

  server.begin();


  Wire.begin();
  mpu6050.begin(0.996);
  // init_MPU6050();

}

void loop() {
  server.handleClient();

  if (micros() - v_timer > 500){
    leftStep += constrain(targetLeftStep - leftStep, -10, 10); 
    rightStep += constrain(targetRightStep - rightStep, -10, 10); 

    leftAngle += constrain(targetLeftAngle - leftAngle, -0.2, 0.2);
    rightAngle += constrain(targetRightAngle - rightAngle, -0.2, 0.2);
    myservo.write(RSERVO, 180 - rightAngle);
    myservo.write(LSERVO, leftAngle);
    offsetAngle = mymap((leftAngle + rightAngle)/2, 0, 30, -1.2, 1.8);
    v_timer = micros();
  }

  mpu6050.update();

  caculate_pid_left();
  caculate_pid_right();
  lastError = error;
  lastLastError = lastError;

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
    Serial.print(' ');
    Serial.print(mpu6050.getAnglePitch());
    Serial.print('\t');
    Serial.print(leftStep);
    Serial.print('\t');
    Serial.println(motLeft.getStep());

  while(micros() - sampling_timer < 2000); //
  sampling_timer = micros(); //Reset the sampling timer  
}

void caculate_pid_left (){

  error = mpu6050.getAnglePitch() - useOffsetAngle;

  if (error > -30 && error < 30){
    Ml = Km*(leftStep - motLeft.getStep());
    Cl = Kc*motLeft.speed;
    Tl += Kt*(leftStep - motLeft.getStep());
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

    Mr = Km*(rightStep - motRight.getStep());
    Cr = Kc*motRight.speed;
    Tr += Kt*(rightStep - motRight.getStep());
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