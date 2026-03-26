#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
  int value;
} struct_message;

struct_message incomingData;

void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(incomingData));
  Serial.print("Received: ");
  Serial.println(incomingData.value);
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
}

void loop() {}