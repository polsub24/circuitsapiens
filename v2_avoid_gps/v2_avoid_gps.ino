#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// ---------------- GPS ----------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// Flag to allow movement
bool gpsReady = false;

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
float distBuffer[N];
int idx = 0;

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
  int tof = (m.RangeStatus!=4)? m.RangeMilliMeter/10 : 999;
  return movingAverage(tof);
}

float getDistanceLeft(){
  return readUltrasonic(TRIG_L, ECHO_L);
}

float getDistanceRight(){
  return readUltrasonic(TRIG_R, ECHO_R);
}

// ---------------- GPS CHECK ----------------
void checkGPS(){

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    gpsReady = true;

    Serial.print("GPS FIXED | Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" | Lon: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Waiting for GPS fix...");
  }
}

// ---------------- CONTROL LOGIC ----------------
void controlLogic(float dist){

  float leftDist = getDistanceLeft();
  delay(10);
  float rightDist = getDistanceRight();

  Serial.print("Front: "); Serial.print(dist);
  Serial.print(" | Left: "); Serial.print(leftDist);
  Serial.print(" | Right: "); Serial.println(rightDist);

  if(dist < 10){
    stopMotors();
    delay(200);
    return;
  }

  if(dist < 400){

    if(leftDist > rightDist && leftDist > 15){
      turnLeft();
    }
    else if(rightDist > 15){
      turnRight();
    }
    else{
      stopMotors();
    }

    delay(200);
    return;
  }

  moveForward();
}

// ---------------- SETUP ----------------
void setup(){

  Serial.begin(115200);

  // GPS UART (RX=16, TX=17)
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  pinMode(TRIG_L,OUTPUT); pinMode(ECHO_L,INPUT);
  pinMode(TRIG_R,OUTPUT); pinMode(ECHO_R,INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  Wire.begin(21,22);
  lox.begin();

  analogWrite(ENA, 100);
  analogWrite(ENB, 100);

  Serial.println("System Initialized. Waiting for GPS...");
}

// ---------------- LOOP ----------------
void loop(){

  checkGPS();

  // 🚫 DO NOT MOVE until GPS fix
  if(!gpsReady){
    stopMotors();
    delay(500);
    return;
  }

  float dist = getDistanceFront();

  controlLogic(dist);

  delay(50);
}