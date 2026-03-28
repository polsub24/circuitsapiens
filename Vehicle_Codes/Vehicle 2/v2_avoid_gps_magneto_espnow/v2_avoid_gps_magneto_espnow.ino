#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <esp_now.h>
#include <QMC5883LCompass.h>
#include "esp_wifi.h"

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

// ---------------- MONITOR ----------------
unsigned long lastReceiveTime = 0;

// ---------------- RECEIVE ----------------
void onReceive(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if(len == sizeof(VehicleData)){
    memcpy(&leaderData, data, sizeof(leaderData));
    lastReceiveTime = millis();

    Serial.println("DATA RECEIVED");
  }
}

// ---------------- MOTOR ----------------
void moveForward(){
  Serial.println("FORWARD");
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
}

void stopMotors(){
  Serial.println("STOP");
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
}

void turnRight(){
  Serial.println("RIGHT");
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
}

void turnLeft(){
  Serial.println("LEFT");
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
}

// ---------------- SENSORS ----------------
float getDistanceFront(){
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m,false);
  return (m.RangeStatus!=4)? m.RangeMilliMeter/10 : 999;
}

float getHeading(){
  compass.read();
  return compass.getAzimuth();
}

// ---------------- FOLLOW LOGIC (FIXED) ----------------
void followLeader(){

  float myHeading = getHeading();
  float error = leaderData.heading - myHeading;

  // Normalize
  if(error > 180) error -= 360;
  if(error < -180) error += 360;

  Serial.print("Error: ");
  Serial.println(error);

  float Kp = 1.0;  // tuning parameter
  int baseSpeed = 120;

  // ---------------- HEADING CONTROL ----------------
  if(abs(error) > 25){
    int turnSpeed = constrain(abs(error) * Kp, 80, 150);

    analogWrite(ENA, turnSpeed);
    analogWrite(ENB, turnSpeed);

    if(error > 0){
      turnRight();
    } else {
      turnLeft();
    }

    delay(40); // stabilize
    return;
  }

  // ---------------- FORWARD MOVEMENT ----------------
  analogWrite(ENA, baseSpeed);
  analogWrite(ENB, baseSpeed);

  float dist = getDistanceFront();

  Serial.print("Distance: ");
  Serial.println(dist);

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

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  analogWrite(ENA, 120);
  analogWrite(ENB, 120);

  Wire.begin(21,22);
  lox.begin();

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  compass.init();

  // ---------------- ESP-NOW ----------------
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP INIT FAILED");
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

  // ---------------- FAILSAFE ----------------
  if(millis() - lastReceiveTime > 2000){
    Serial.println("LOST LEADER");
    stopMotors();
    delay(100);
    return;
  }

  // ---------------- MAIN ----------------
  if(gpsLocked && leaderData.gpsValid){
    followLeader();
  }
  else{
    Serial.println("WAITING...");
    stopMotors();
  }

  delay(50);
}