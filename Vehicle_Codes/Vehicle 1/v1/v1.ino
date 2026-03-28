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
#define TRIG_L 5
#define ECHO_L 18

#define TRIG_R 13
#define ECHO_R 14

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
  unsigned long timestamp;   // ADD THIS
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

bool isDataFresh(){
  return (millis() - otherVehicle.timestamp) < 200;
}

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
long readUltrasonic(int trig, int echo){
  digitalWrite(trig,LOW); delayMicroseconds(2);
  digitalWrite(trig,HIGH); delayMicroseconds(10);
  digitalWrite(trig,LOW);

  long duration = pulseIn(echo,HIGH,20000);
  return duration * 0.034 / 2;
}

float getDistance(){
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m,false);

  int tof = (m.RangeStatus!=4)? m.RangeMilliMeter/10 : 999;
  return movingAverage(tof);
}

float getDistanceLeft(){
  return readUltrasonic(TRIG_L, ECHO_L);
}

float getDistanceRight(){
  return readUltrasonic(TRIG_R, ECHO_R);
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

// Receive callback (already updated)
void onReceive(const esp_now_recv_info *info, const uint8_t *data, int len){
  if(len == sizeof(VehicleData)){
    memcpy(&otherVehicle, data, sizeof(otherVehicle));
  }
}

// ✅ SEND CALLBACK (PLACE HERE, OUTSIDE setup)
void onSend(const wifi_tx_info_t *info, esp_now_send_status_t status){
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "TX OK" : "TX FAIL");
}


void sendData(){
  esp_now_send(peerMAC,(uint8_t*)&myData,sizeof(myData));
}

// ---------------- CONTROL ----------------
void controlLogic(float dist){

  float speed = getSpeed();

  // ✅ DATA VALIDITY CHECK
  if(!isDataFresh()){
    stopMotors();
    return;
  }

  float relSpeed = speed - otherVehicle.speed;
  float ttc = (relSpeed > 0.1) ? dist / relSpeed : 999;

  if(emergencyFlag || otherVehicle.emergency){
    stopMotors();
    return;
  }

  if(!deviated && dist < 20){
    pathHeading = currentHeading;
    deviated = true;
  }

  if(ttc < 2 || dist < 20){

    stopMotors();

    float headingDiff = otherVehicle.heading - currentHeading;

    if(headingDiff > 180) headingDiff -= 360;
    if(headingDiff < -180) headingDiff += 360;

    // ✅ SIDE SENSORS
    float leftDist = getDistanceLeft();
    delay(10);
    float rightDist = getDistanceRight();

    // 🚫 BOTH SIDES BLOCKED
    if(leftDist < 10 && rightDist < 10){
      stopMotors();
      return;
    }

    // ✅ SMART DECISION (HEADING + SPACE)
    if(headingDiff > 0 && rightDist > 15){
      turnRight();
    }
    else if(headingDiff < 0 && leftDist > 15){
      turnLeft();
    }
    else{
      // fallback: choose freer side
      if(leftDist > rightDist) turnLeft();
      else turnRight();
    }
  }

  else if(deviated){
    headingPID(pathHeading);
    if(abs(currentHeading - pathHeading) < 5) deviated = false;
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
  pinMode(TRIG_L,OUTPUT); pinMode(ECHO_L,INPUT);
  pinMode(TRIG_R,OUTPUT); pinMode(ECHO_R,INPUT);

  Wire.begin(21,22);

  // MPU wake
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  compass.init();
  lox.begin();

  gpsSerial.begin(9600,SERIAL_8N1,16,17);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
  Serial.println("ESP-NOW INIT FAILED");
  return;
  }

  // Receive callback
  esp_now_register_recv_cb(onReceive);
  esp_now_register_send_cb(onSend);

  // Add peer
  esp_now_peer_info_t peer;
  memcpy(peer.peer_addr, peerMAC, 6);
  memset(&otherVehicle, 0, sizeof(otherVehicle));
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
  Serial.println("Peer Add Failed");
}

  lastTime = millis();
}

// ---------------- LOOP ----------------
void loop(){

  updateMPU();

  while(gpsSerial.available()){
    gps.encode(gpsSerial.read());
  }
  float dist = getDistance();
  controlLogic(dist);

  if(gps.location.isValid()){
    myData.lat = gps.location.lat();
    myData.lng = gps.location.lng();
  }

  myData.heading = currentHeading;
  myData.speed = getSpeed();
  myData.distance = dist;
  myData.emergency = emergencyFlag;
  myData.vehicleID = vehicleID;

  myData.timestamp = millis();
  sendData();

  delay(50);
}