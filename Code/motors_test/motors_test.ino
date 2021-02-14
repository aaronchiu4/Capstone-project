#include <Servo.h>
#include <SoftwareSerial.h>

Servo leftmotor;  // create servo object to control a servo
Servo rightmotor;

#define motormax 1700 // max motor speed per cycle - for motor speed
#define brake 1550 // brake motors - for motor speed
#define motormin 1400 //min motor speed (negative) - for motor speed
#define acceleration 20 // speed of motorspeed value increase/decrease

int horizontalpot = 2;  // analog pin used to connect the potentiometer
int verticalpot = 1;
int horizontal;
int vertical;
int dir; ///directon
int motorspeed;
int temp;


void setup() {
  
  leftmotor.attach(4);  // attaches the servo on pin 4 to the servo object
  rightmotor.attach(3);
  Serial.begin(115200);
  motorspeed = brake;
  temp = 0;

}

void loop() {
  horizontal = analogRead(horizontalpot);            // reads the value of the potentiometer (value between 0 and 1023)
  vertical = analogRead(verticalpot);

  
  horizontal = map(horizontal, 0, 1024, 1000, 2000);    
  vertical = map(vertical, 0, 1024, 1000, 2000);

  Serial.print("horizontal: ");
  Serial.print(horizontal);
  Serial.print("  vertical: ");
  Serial.print(vertical);
  Serial.print("  motorspeed: ");
  Serial.println(motorspeed);


  if (horizontal > 1700){
    speedup();
    left(motorspeed);
  }else if (horizontal < 1300){
    speedup();
    right(motorspeed);
  }else if (vertical > 1700){
    speedup();
    fb(motorspeed);
  }else if (vertical < 1300){
    rev();
    fb(motorspeed);
  }else{
    stp();
    fb(motorspeed);
  }


  Serial.print("motorspeed: ");
  Serial.println(motorspeed);

  

  delay(15);                           // waits for the servo to get there
}

void fb(int motorspeed){
  
  leftmotor.writeMicroseconds(motorspeed);
  rightmotor.writeMicroseconds(motorspeed); 
  Serial.print("motorspeed: ");
  Serial.println(motorspeed); 
  Serial.println("forward");

}

void left(int motorspeed){

  leftmotor.writeMicroseconds(motorspeed);
  Serial.println("right");

  
}

void right(int motorspeed){

  rightmotor.writeMicroseconds(motorspeed); 
  Serial.println("left");  
}

void speedup(){
  if (motorspeed < motormax){
    motorspeed = motorspeed + acceleration;
    delay(15);
  }
}

void rev(){
  if (motorspeed > motormin){
    motorspeed = motorspeed - acceleration;
    delay(15);
  }
}

void stp(){
  motorspeed = brake;
  Serial.println("brake"); 
}

