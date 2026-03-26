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

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ---------------- DATA ----------------
typedef struct {
  float lat;
  float lon;
  float heading;
  bool gpsValid;
} VehicleData;

VehicleData leaderData;

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
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
}

void turnLeft(){
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
}

// ---------------- RECEIVE (FIXED) ----------------
void onReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {

  if(len == sizeof(VehicleData)){
    memcpy(&leaderData, data, sizeof(leaderData));

    // Debug print (optional but useful)
    Serial.print("RX Heading: ");
    Serial.print(leaderData.heading);
    Serial.print(" | GPS Valid: ");
    Serial.println(leaderData.gpsValid);
  }
}

// ---------------- DISTANCE ----------------
float getDistanceFront(){
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m,false);
  return (m.RangeStatus!=4)? m.RangeMilliMeter/10 : 999;
}

// ---------------- HEADING ----------------
float getHeading(){
  compass.read();
  return compass.getAzimuth();
}

// ---------------- FOLLOW LOGIC ----------------
void followLeader(){

  if(!leaderData.gpsValid){
    stopMotors();
    return;
  }

  float myHeading = getHeading();
  float error = leaderData.heading - myHeading;

  // Normalize error
  if(error > 180) error -= 360;
  if(error < -180) error += 360;

  Serial.print("Heading Error: ");
  Serial.println(error);

  // Alignment phase
  if(abs(error) > 10){
    if(error > 0) turnRight();
    else turnLeft();
    return;
  }

  // Distance control
  float dist = getDistanceFront();

  if(dist > 20){
    moveForward();
  } else {
    stopMotors();
  }
}

// ---------------- SETUP ----------------
void setup(){
  Serial.begin(115200);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  // PWM using analogWrite ONLY
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  Wire.begin(21,22);
  lox.begin();

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  compass.init();

  // ESP-NOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  Serial.println("V2 READY");
}

// ---------------- LOOP ----------------
void loop(){

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid() && gps.satellites.value() >= 4) {
    gpsLocked = true;
  }

  if(gpsLocked && leaderData.gpsValid){
    followLeader();
  }
  else{
    stopMotors();
  }

  delay(50);
}