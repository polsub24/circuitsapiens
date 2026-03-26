#include <HardwareSerial.h>

HardwareSerial gpsSerial(2);

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  Serial.println("START");
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    Serial.write(c);
  }
}