#include <SoftwareSerial.h>

int temp = 0;

void setup() {
  Serial.begin(115200);
  

}

void loop() {
  
  if(Serial.available() > 0) {
    temp = Serial.read();
    Serial.println(temp);
  }

}
