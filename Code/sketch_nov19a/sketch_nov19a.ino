#include <Servo.h>
#include <SoftwareSerial.h>

Servo leftmotor;  // create servo object to control a servo
Servo rightmotor;

int horizontalpot = 0;  // analog pin used to connect the potentiometer
int verticalpot = 1;
int horizontalpercent;    //hrizontal percentage left = 1024(100%)
int vertical;

void setup() {
  
  leftmotor.attach(4);  // attaches the servo on pin 4 to the servo object
  rightmotor.attach(5);
  Serial.begin(115200);
}

void loop() {
  horizontalpercent = analogRead(horizontalpot);            // reads the value of the potentiometer (value between 0 and 1023)
  vertical = analogRead(verticalpot);

  Serial.print("1 horizontalpercentage: ");
  Serial.print(horizontalpercent);
  Serial.print("  vertical: ");
  Serial.println(vertical);
  
  horizontalpercent = map(horizontalpercent, 0, 1024, 1000, 2000);    
  vertical = map(vertical, 0, 1024, 1000, 2000); //reduce acceleration

  
  Serial.print("2 horizontalpercentage: ");
  Serial.print(horizontalpercent);
  Serial.print("  vertical: ");
  Serial.println(vertical);
  
  // determines direction 
   if (horizontalpercent < 1400){
    left(vertical);
   }else if (horizontalpercent >1600){
    right(vertical);
   }else{
    fb(vertical);                  
   }
  
  //most basic form of control
  /*(if (horizontalpercent < 1400){
    
    leftmotor.writeMicroseconds(vertical);
    Serial.println("IF 1");
  }else if (horizontalpercent >1600){
    rightmotor.writeMicroseconds(vertical); 
    Serial.println("IF 2");         
  }else{
    leftmotor.writeMicroseconds(vertical);
    rightmotor.writeMicroseconds(vertical); 
    Serial.println("IF 3");
  }*/

  delay(15);                           // waits for the servo to get there
}

void fb(int vertical){
  leftmotor.writeMicroseconds(vertical);
  rightmotor.writeMicroseconds(vertical); 
  Serial.println("IF 3");
}

void left(int vertical){
  leftmotor.writeMicroseconds(vertical);
  Serial.println("IF 1");
}

void right(int vertical){
  rightmotor.writeMicroseconds(vertical); 
  Serial.println("IF 2");   
}

