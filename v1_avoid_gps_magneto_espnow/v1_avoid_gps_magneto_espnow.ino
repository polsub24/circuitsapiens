#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <QMC5883LCompass.h>

// ---------------- GPS ----------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
bool gpsLocked = false;

// ---------------- COMPASS ----------------
QMC5883LCompass compass;

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

// ---------------- ESP-NOW ----------------
uint8_t peerMAC[] = {0xB0,0xA7,0x32,0xDB,0xB5,0x14}; // PUT V2 MAC

typedef struct {
  float lat;
  float lon;
  float heading;
  bool gpsValid;
} VehicleData;

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

// ---------------- ULTRASONIC ----------------
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
  return (m.RangeStatus!=4)? m.RangeMilliMeter/10 : 999;
}

// ---------------- COMPASS ----------------
float getHeading(){
  compass.read();
  return compass.getAzimuth();
}

// ---------------- ESP SEND ----------------
void sendData(){
  VehicleData data;
  data.lat = gps.location.lat();
  data.lon = gps.location.lng();
  data.heading = getHeading();
  data.gpsValid = gps.location.isValid();

  esp_now_send(peerMAC, (uint8_t*)&data, sizeof(data));
}

// ---------------- CONTROL ----------------
void controlLogic(float dist){
  float leftDist = readUltrasonic(TRIG_L, ECHO_L);
  float rightDist = readUltrasonic(TRIG_R, ECHO_R);

  if(dist < 10){
    stopMotors();
    return;
  }

  if(dist < 400){
    if(leftDist > rightDist) turnLeft();
    else turnRight();
    return;
  }

  moveForward();
}

// ---------------- SETUP ----------------
void setup(){
  Serial.begin(115200);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  pinMode(TRIG_L,OUTPUT); pinMode(ECHO_L,INPUT);
  pinMode(TRIG_R,OUTPUT); pinMode(ECHO_R,INPUT);

  // PWM using analogWrite ONLY
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  analogWrite(ENA, 80);
  analogWrite(ENB, 255);

  Wire.begin(21,22);
  lox.begin();

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  compass.init();

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_now_init();

  esp_now_peer_info_t peer;
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("V1 READY");
}

// ---------------- LOOP ----------------
void loop(){

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid() && gps.satellites.value() >= 4) {
    gpsLocked = true;
  }

  if(gpsLocked){
    float dist = getDistanceFront();
    controlLogic(dist);
    sendData();
  }
  else{
    stopMotors();
  }

  delay(50);
}