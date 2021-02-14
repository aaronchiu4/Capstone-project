#include <SoftwareSerial.h>

char temp = "0.57483";

void setup() {
  Serial.begin(115200);

}

void loop() {
  Serial.write(temp);
  Serial.println(temp);
  delay(100);
}
