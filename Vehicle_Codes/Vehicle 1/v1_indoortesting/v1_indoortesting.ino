#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- OLED ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

String oledStatus="INIT", oledTX="-", oledRX="-";
int oledRSSI=0;

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

Adafruit_VL53L0X lox;

// ---------------- FILTER ----------------
#define N 5
float distBuffer[N]={0};
int idx=0;

// ---------------- V2V ----------------
typedef struct {
  float frontDist;
  int state;
} VehicleData;

VehicleData sendData, recvData;

// 🔴 V2 MAC
uint8_t peerMAC[] = {0x14,0x33,0x5C,0x0B,0x23,0xAC};

// ---------------- MOTOR ----------------
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
  digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
}
void turnLeft(){
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
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
long readUltrasonic(int trig,int echo){
  digitalWrite(trig,LOW); delayMicroseconds(2);
  digitalWrite(trig,HIGH); delayMicroseconds(10);
  digitalWrite(trig,LOW);
  long d=pulseIn(echo,HIGH,20000);
  return d*0.034/2;
}

float getDistanceFront(){
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m,false);
  int tof=(m.RangeStatus!=4)?m.RangeMilliMeter/10:999;
  return movingAverage(tof);
}

float getDistanceLeft(){ return readUltrasonic(TRIG_L,ECHO_L); }
float getDistanceRight(){ return readUltrasonic(TRIG_R,ECHO_R); }

// ---------------- OLED ----------------
void updateOLED(float dist,int state){
  display.clearDisplay();
  display.setCursor(0,0); display.print("F:"); display.println(dist);
  display.setCursor(0,10); display.print("S:"); display.println(state);
  display.setCursor(0,20); display.print("TX:"); display.println(oledTX);
  display.setCursor(0,30); display.print("RX:"); display.println(oledRX);
  display.setCursor(0,40); display.print("RSSI:"); display.println(oledRSSI);
  display.setCursor(0,50); display.print("Act:"); display.println(oledStatus);
  display.display();
}

// ---------------- ESP-NOW ----------------
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status){
  oledTX = (status==ESP_NOW_SEND_SUCCESS)?"OK":"FAIL";
}

void OnDataRecv(const esp_now_recv_info_t *info,const uint8_t *data,int len){
  memcpy(&recvData,data,sizeof(recvData));
  oledRX=String(recvData.frontDist,1)+","+String(recvData.state);
  oledRSSI=info->rx_ctrl->rssi;
}

void sendVehicleData(float dist,int state){
  sendData.frontDist=dist;
  sendData.state=state;
  esp_now_send(peerMAC,(uint8_t*)&sendData,sizeof(sendData));
}

// ---------------- CONTROL ----------------
void controlLogic(float dist){

  float left=getDistanceLeft();
  delay(10);
  float right=getDistanceRight();

  if(dist<10){
    stopMotors(); oledStatus="STOP";
    sendVehicleData(dist,0);
    updateOLED(dist,0);
    return;
  }

  if(dist<400){
    if(left>right && left>15){ turnLeft(); oledStatus="LEFT"; }
    else if(right>15){ turnRight(); oledStatus="RIGHT"; }
    else{ stopMotors(); oledStatus="BLOCK"; }

    sendVehicleData(dist,2);
    updateOLED(dist,2);
    return;
  }

  if(recvData.frontDist<50 && recvData.state==1){
    stopMotors(); oledStatus="V2V";
    sendVehicleData(dist,0);
    updateOLED(dist,0);
    return;
  }

  moveForward(); oledStatus="FWD";
  sendVehicleData(dist,1);
  updateOLED(dist,1);
}

// ---------------- SETUP ----------------
void setup(){
  Serial.begin(115200);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  pinMode(TRIG_L,OUTPUT); pinMode(ECHO_L,INPUT);
  pinMode(TRIG_R,OUTPUT); pinMode(ECHO_R,INPUT);

  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);

  Wire.begin(21,22);
  lox.begin();

  analogWrite(ENA,80);
  analogWrite(ENB,255);

  display.begin(SSD1306_SWITCHCAPVCC,0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  WiFi.mode(WIFI_STA);
  esp_now_init();

  esp_now_peer_info_t peerInfo={};
  memcpy(peerInfo.peer_addr,peerMAC,6);
  esp_now_add_peer(&peerInfo);

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
}

// ---------------- LOOP ----------------
void loop(){
  float dist=getDistanceFront();
  controlLogic(dist);
  delay(50);
}