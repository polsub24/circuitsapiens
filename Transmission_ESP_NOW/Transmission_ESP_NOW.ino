#include <esp_now.h>
#include <WiFi.h>

uint8_t receiverMAC[] = {0x44, 0x1D, 0x64, 0xBD, 0x89, 0xB0};

typedef struct struct_message {
  int value;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  myData.value = random(0, 100);

  esp_now_send(receiverMAC, (uint8_t *) &myData, sizeof(myData));

  Serial.print("Sent: ");
  Serial.println(myData.value);

  delay(2000);
}