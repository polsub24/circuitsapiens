#include <QMC5883LCompass.h>
QMC5883LCompass compass;

int minX=9999, maxX=-9999;
int minY=9999, maxY=-9999;

void setup(){
  Serial.begin(115200);
  compass.init();
}

void loop(){
  compass.read();
  int x = compass.getX();
  int y = compass.getY();

  minX = min(minX, x); maxX = max(maxX, x);
  minY = min(minY, y); maxY = max(maxY, y);

  Serial.printf("X:%d Y:%d | minX:%d maxX:%d minY:%d maxY:%d\n",
                x,y,minX,maxX,minY,maxY);
  delay(100);
}