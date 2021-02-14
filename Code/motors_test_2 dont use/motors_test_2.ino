#include <Servo.h>
#include <SoftwareSerial.h>

Servo leftmotor;  // create servo object to control a motor driver
Servo rightmotor;

//set variables and limits below
#define motormax 1700 // max motor speed per cycle - for motor speed
#define brake 1550 // brake motors - for motor speed
#define motormin 1400 //min motor speed (negative) - for motor speed
#define acceleration 20 // speed of motorspeed value increase/decrease




int dir; ///directon
int motorspeed;
int temp;
int dirangle; //direction angle 0 - 360
int targetangle; //desired angle
bool targetposition; // is the cart at the target position

void setup() {
  
  leftmotor.attach(4);  // attaches the servo on pin 4 to the servo object
  rightmotor.attach(5);
  Serial.begin(115200);
  motorspeed = brake;
  temp = 0;

  dirangle = 0;
  targetangle = 0;
  targetposition = false;
}

void loop() {

  //for testing, should be read from code in loop for intergration
  int dirangle = 0; //read from compass
  int targetangle = 0; // angle from phone gps
  bool targetposition = false; // 0=false 1=true 


  dirangle = 0; //read from compass
  targetangle = 0; // angle from phone gps
  targetposition = false;

  Serial.print("dirangle: ");
  Serial.print(dirangle);
  Serial.print("  targetangle: ");
  Serial.print(targetangle);
  Serial.print("  targetposition: ");
  Serial.println(targetposition);
 
  
  if (targetposition == true){
    stp();
    fb(motorspeed);
  }else if ((targetangle-3<dirangle<targetangle+3) && targetposition == false){
    speedup();
    fb(motorspeed);
    Serial.println("forward");
  }else if (dirangle > targetangle){
    speedup();
    left(motorspeed);
  }else if (dirangle < targetangle){
    speedup();
    right(motorspeed);
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
  Serial.println("FB");

}

void left(int motorspeed){

  leftmotor.writeMicroseconds(motorspeed);
  Serial.println("LEFT");

  
}

void right(int motorspeed){

  rightmotor.writeMicroseconds(motorspeed); 
  Serial.println("RIGHT");  
}
// increase speed
void speedup(){
  if (motorspeed < motormax){
    motorspeed = motorspeed + acceleration;
    delay(15);
  }
}

//increase speed in reverse
void revspeedup(){
  if (motorspeed > motormin){
    motorspeed = motorspeed - acceleration;
    delay(15);
  }
}

//sets wheels to brake
void stp(){
  motorspeed = brake;
}

