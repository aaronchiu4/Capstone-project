#include <SoftwareSerial9.h>

#define tx 5
#define rx 4

SoftwareSerial9 serial1(rx,tx);

void setup() {
  
  serial1.begin(26315);
  //Serial.begin(115200);
}

void loop() {
  //preamble
  serial1.write9(0x100);
  //data
  serial1.write9(0x0FD);
  serial1.write9(0x0F8);
  //data repeat
  serial1.write9(0x0FD);
  serial1.write9(0x0F8);
  //motor enable signal
  serial1.write9(0x055);

  //Serial.println("loop");

}


