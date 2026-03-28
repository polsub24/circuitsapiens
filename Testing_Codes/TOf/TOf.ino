#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // SDA, SCL

  Serial.println("VL53L0X Test");

  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while (1);
  }

  Serial.println("Sensor ready!");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {  // valid measurement
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println("Out of range");
  }

  delay(500);
}