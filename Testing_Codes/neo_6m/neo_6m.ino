#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

unsigned long lastDisplay = 0;

// Toggle this to enable/disable raw data
bool showRaw = true;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  Serial.println("GPS6MV2 HW-248 Advanced Monitor");
}

void loop() {

  while (gpsSerial.available()) {
    char c = gpsSerial.read();

    // Feed parser
    gps.encode(c);

    // Print raw NMEA if enabled
    if (showRaw) {
      Serial.write(c);
    }
  }

  // Structured output every 1 second
  if (millis() - lastDisplay > 1000) {
    lastDisplay = millis();

    Serial.println("\n------ PARSED DATA ------");

    // Satellites
    Serial.print("Satellites: ");
    if (gps.satellites.isValid())
      Serial.println(gps.satellites.value());
    else
      Serial.println("Searching...");

    // HDOP
    Serial.print("HDOP: ");
    if (gps.hdop.isValid())
      Serial.println(gps.hdop.hdop());
    else
      Serial.println("N/A");

    // Location
    if (gps.location.isValid()) {
      Serial.print("Lat: ");
      Serial.println(gps.location.lat(), 6);

      Serial.print("Lng: ");
      Serial.println(gps.location.lng(), 6);

      Serial.print("Map: https://maps.google.com/?q=");
      Serial.print(gps.location.lat(), 6);
      Serial.print(",");
      Serial.println(gps.location.lng(), 6);

      Serial.println("Status: FIX ACQUIRED ✅");
    } else {
      Serial.println("Location: Not Available");
      Serial.println("Status: SEARCHING ⏳");
    }

    // Time
    Serial.print("Time (UTC): ");
    if (gps.time.isValid()) {
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.println(gps.time.second());
    } else {
      Serial.println("Not synced");
    }

    Serial.println("--------------------------\n");
  }
}