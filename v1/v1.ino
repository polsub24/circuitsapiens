#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <QMC5883LCompass.h>

// ---------------- MOTOR ----------------
#define IN1 25
#define IN2 26
#define IN3 33
#define IN4 32

// ---------------- ULTRASONIC ----------------
#define TRIG 5
#define ECHO 18

// ---------------- MPU ----------------
#define MPU_ADDR 0x68


// ---------------- OBJECTS ----------------
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
QMC5883LCompass compass;

// ---------------- ESP-NOW ----------------
uint8_t peerMAC[] = {0xB0, 0xA7, 0x32, 0xDB, 0xB5, 0x14}; // CHANGE

typedef struct {
  float lat, lng, heading, speed;
  int distance;
  bool emergency;
  uint8_t vehicleID;
} VehicleData;

VehicleData myData, otherVehicle;

// ---------------- GLOBALS ----------------
float currentHeading = 0, pathHeading = 0;
bool deviated = false;

float velocity = 0;
unsigned long lastTime = 0;

float filteredAx = 0;
float alpha = 0.7;

#define N 5
float distBuffer[N];
int idx = 0;

// ------------MAGNETOMETER----------------
float offsetX = -150;
float offsetY = -1225;

float scaleX = 1.425;
float scaleY = 0.77;

bool emergencyFlag = false;
uint8_t vehicleID = 1;

// ---------------- MOTOR ----------------
void moveForward() {
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
}
void stopMotors() {
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
}
void turnLeft() {
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
}
void turnRight() {
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
}

// ---------------- FILTERS ----------------
float movingAverage(float val){
  distBuffer[idx++] = val;
  if(idx>=N) idx=0;
  float sum=0;
  for(int i=0;i<N;i++) sum+=distBuffer[i];
  return sum/N;
}
float lowPass(float input){
  filteredAx = alpha*filteredAx + (1-alpha)*input;
  return filteredAx;
}

// ---------------- MPU ----------------
void updateMPU(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR,6,true);

  int16_t ax = Wire.read()<<8 | Wire.read();
  int16_t ay = Wire.read()<<8 | Wire.read();
  int16_t az = Wire.read()<<8 | Wire.read();

  float ax_mps2 = (ax/16384.0)*9.81;
  ax_mps2 = lowPass(ax_mps2);

  unsigned long now = millis();
  float dt = (now-lastTime)/1000.0;
  lastTime = now;

  velocity += ax_mps2*dt;

  if(abs(ax_mps2)<0.05) velocity*=0.9;

  if(ax_mps2 < -2) emergencyFlag=true;
  else emergencyFlag=false;

  // store for heading
  currentHeading = getHeading(ax,ay,az);
}

// ---------------- MAGNETOMETER ----------------
float getHeading(int16_t ax,int16_t ay,int16_t az){

  float roll = atan2(ay,az);
  float pitch = atan(-ax / sqrt(ay*ay + az*az));

  compass.read();

  float mx = compass.getX();
  float my = compass.getY();
  float mz = compass.getZ();

  // ✅ Calibration correction
  mx = (mx - offsetX) * scaleX;
  my = (my - offsetY) * scaleY;

  float Xh = mx*cos(pitch) + mz*sin(pitch);
  float Yh = mx*sin(roll)*sin(pitch) + my*cos(roll) - mz*sin(roll)*cos(pitch);

  float heading = atan2(Yh,Xh)*180/PI;
  if(heading < 0) heading += 360;

  return heading;
}

// ---------------- DISTANCE ----------------
long readUltrasonic(){
  digitalWrite(TRIG,LOW); delayMicroseconds(2);
  digitalWrite(TRIG,HIGH); delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  long duration = pulseIn(ECHO,HIGH,20000);
  return duration*0.034/2;
}

float getDistance(){
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m,false);

  int tof = (m.RangeStatus!=4)? m.RangeMilliMeter/10 : 999;
  int ultra = readUltrasonic();

  float fused = (0.9*tof + 0.6*ultra)/(0.9+0.6);
  return movingAverage(fused);
}

// ---------------- SPEED ----------------
float getSpeed(){
  float gpsSpeed = gps.speed.isValid()? gps.speed.mps():0;
  return 0.7*gpsSpeed + 0.3*velocity;
}

// ---------------- PID ----------------
float Kp=1.5,Kd=0.2;
float prevError=0;

void headingPID(float target){
  float error = target-currentHeading;

  if(error>180) error-=360;
  if(error<-180) error+=360;

  float derivative = error-prevError;
  float output = Kp*error + Kd*derivative;
  prevError = error;

  if(output>5) turnLeft();
  else if(output<-5) turnRight();
  else moveForward();
}

// ---------------- ESP-NOW ----------------
void onReceive(const uint8_t *mac,const uint8_t *data,int len){
  memcpy(&otherVehicle,data,sizeof(otherVehicle));
}

void sendData(){
  esp_now_send(peerMAC,(uint8_t*)&myData,sizeof(myData));
}

// ---------------- CONTROL ----------------
void controlLogic(){

  float dist = getDistance();
  float speed = getSpeed();
  float relSpeed = speed - otherVehicle.speed;

  float ttc = (relSpeed>0)? dist/relSpeed : 999;

  if(emergencyFlag || otherVehicle.emergency){
    stopMotors();
    return;
  }

  if(!deviated && dist<20){
    pathHeading = currentHeading;
    deviated = true;
  }

  if(ttc<2 || dist<20){

    stopMotors();

    if(vehicleID < otherVehicle.vehicleID) turnLeft();
    else turnRight();

  }
  else if(deviated){
    headingPID(pathHeading);
    if(abs(currentHeading-pathHeading)<5) deviated=false;
  }
  else{
    moveForward();
  }
}

// ---------------- SETUP ----------------
void setup(){

  Serial.begin(115200);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(TRIG,OUTPUT); pinMode(ECHO,INPUT);

  Wire.begin(21,22);

  // MPU wake
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  compass.init();
  lox.begin();

  gpsSerial.begin(9600,SERIAL_8N1,16,17);

  WiFi.mode(WIFI_STA);
  esp_now_init();

  // Receive callback
  esp_now_register_recv_cb(onReceive);

  // ✅ ADD THIS BLOCK
  esp_now_register_send_cb([](const uint8_t*, esp_now_send_status_t s){
    Serial.println(s == ESP_NOW_SEND_SUCCESS ? "TX OK" : "TX FAIL");
  });

  // Add peer
  esp_now_peer_info_t peer;
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  lastTime = millis();
}

// ---------------- LOOP ----------------
void loop(){

  updateMPU();

  while(gpsSerial.available()){
    gps.encode(gpsSerial.read());
  }

  controlLogic();

  if(gps.location.isValid()){
    myData.lat = gps.location.lat();
    myData.lng = gps.location.lng();
  }

  myData.heading = currentHeading;
  myData.speed = getSpeed();
  myData.distance = getDistance();
  myData.emergency = emergencyFlag;
  myData.vehicleID = vehicleID;

  sendData();

  delay(50);
}