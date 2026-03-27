#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <esp_now.h>
#include <WiFi.h>

// ---------------- MOTOR ----------------
#define IN1 25
#define IN2 26
#define IN3 33
#define IN4 32
#define ENA 27
#define ENB 19

// ---------------- ULTRASONIC ----------------
#define TRIG_L 5
#define ECHO_L 18
#define TRIG_R 13
#define ECHO_R 14

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ---------------- FILTER ----------------
#define N 5
float distBuffer[N] = {0};
int idx = 0;

// ---------------- V2V ----------------
typedef struct {
  float frontDist;
  int state; // 0 stop, 1 move, 2 obstacle
} VehicleData;

VehicleData sendData, recvData;

// 🔴 PUT V2 MAC HERE
uint8_t peerMAC[] = {0xAF,0xF0,0x0F,0x5C,0x41,0xA4};

// ---------------- MOTOR CONTROL ----------------
void moveForward(){
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
}
void stopMotors(){
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
}
void turnRight(){
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
}
void turnLeft(){
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
}

// ---------------- FILTER ----------------
float movingAverage(float val){
  distBuffer[idx++] = val;
  if(idx>=N) idx=0;
  float sum=0;
  for(int i=0;i<N;i++) sum+=distBuffer[i];
  return sum/N;
}

// ---------------- SENSORS ----------------
long readUltrasonic(int trig, int echo){
  digitalWrite(trig,LOW); delayMicroseconds(2);
  digitalWrite(trig,HIGH); delayMicroseconds(10);
  digitalWrite(trig,LOW);
  long duration = pulseIn(echo,HIGH,20000);
  return duration * 0.034 / 2;
}

float getDistanceFront(){
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m,false);
  int tof = (m.RangeStatus!=4)? m.RangeMilliMeter/10 : 999;
  return movingAverage(tof);
}

float getDistanceLeft(){ return readUltrasonic(TRIG_L, ECHO_L); }
float getDistanceRight(){ return readUltrasonic(TRIG_R, ECHO_R); }

// ---------------- ESP-NOW ----------------
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  
  memcpy(&recvData, incomingData, sizeof(recvData));

  Serial.print("Received from: ");
  for(int i=0;i<6;i++){
    Serial.printf("%02X", info->src_addr[i]);
    if(i<5) Serial.print(":");
  }

  Serial.print(" | Dist: ");
  Serial.print(recvData.frontDist);
  Serial.print(" | State: ");
  Serial.println(recvData.state);
}

void sendVehicleData(float dist, int state){
  sendData.frontDist = dist;
  sendData.state = state;
  esp_now_send(peerMAC, (uint8_t *)&sendData, sizeof(sendData));
}

// ---------------- CONTROL ----------------
// ONLY CHANGE IS CONTROL LOGIC DIFFERENCE

void controlLogic(float dist){

  float leftDist = getDistanceLeft();
  delay(10);
  float rightDist = getDistanceRight();

  int myState = 1;

  if(dist < 10){
    stopMotors();
    sendVehicleData(dist, 0);
    delay(200);
    return;
  }

  if(dist < 400){
    myState = 2;

    if(leftDist > rightDist && leftDist > 15) turnLeft();
    else if(rightDist > 15) turnRight();
    else { stopMotors(); myState = 0; }

    sendVehicleData(dist, myState);
    delay(200);
    return;
  }

  // 🔥 STRONGER FOLLOWER LOGIC
  if(recvData.frontDist < 80 || recvData.state == 0){
    stopMotors();
    sendVehicleData(dist, 0);
    delay(100);
    return;
  }

  moveForward();
  sendVehicleData(dist, 1);
}
// ---------------- SETUP ----------------
void setup(){
  Serial.begin(115200);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  pinMode(TRIG_L,OUTPUT); pinMode(ECHO_L,INPUT);
  pinMode(TRIG_R,OUTPUT); pinMode(ECHO_R,INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  Wire.begin(21,22);
  lox.begin();

  analogWrite(ENA, 200);
  analogWrite(ENB, 200);

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_now_init();

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMAC, 6);
  esp_now_add_peer(&peerInfo);

  esp_now_register_recv_cb(OnDataRecv);
}

// ---------------- LOOP ----------------
void loop(){
  float dist = getDistanceFront();
  controlLogic(dist);
  delay(50);
}