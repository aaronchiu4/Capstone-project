// the following code controls the cart using the carts exact location 
// and the location of the gps of the mobile phone.
#include <Servo.h>
#include <SoftwareSerial.h>
#include <math.h>

// create servo object to control a servo
Servo leftmotor;  
Servo rightmotor;

//define specific constants
#define forward 0
#define backward 1
#define turnleft 3
#define turnright 4
#define motormax 1700 // max motor speed per cycle - for motor speed
#define brake 1600 // brake motors - for motor speed
#define motormin 1300 //min motor speed (negative) - for motor speed
#define acceleration 20 // speed of motorspeed value increase/decrease

//initilizing variables 
int horizontalpot = 0;  // analog pin used to connect the potentiometer
int verticalpot = 1;
int horizontal;
int vertical;
int dir; ///directon
int motorspeed;
int temp;

//following is for navigation
int Alat = 0;// latitude of point A
int Along = 0;//longitude of point A
int Blat = 0;//latitude of point B
int Blong = 0;//longitude of point B 
int Dlat = 0;//delta latitude
int Dlong = 0;//delta longitude
int compass = 0;//direction of the cart in angles
int targetangle = 0;//angle from north to the target

void setup() {
  
  leftmotor.attach(4);  // attaches the servo on pin 4 to the servo object
  rightmotor.attach(5);
  Serial.begin(115200);
  motorspeed = brake;
  temp = 0;
 
}

void loop() {

  //following is used to test the function enter arbitrary 
  //coordinates to test mobility functions
  Alat = ;
  Along = ;
  Blat = ;
  Blong = ;
  compass = ;
  Dlat = 4003017*((Blat-Alat)/360);
  Dlong = 4003017*((Blong-Along)/360);
  targetangle = tan(Dlong/Dlat);
  

  //display the values read by the parsed gps data
  Serial.print("A latitude");
  Serial.println(Alat);
  Serial.print("A longitude");
  Serial.println(Along);
  Serial.print("B latitude");
  Serial.println(Blat);
  Serial.print("B longitude");
  Serial.print(Blong);
 
  if (compass < targetangle){
    speedup();
    left(motorspeed);
  }else if (compass > targetangle){
    speedup();
    right(motorspeed);
  }else if (Dlat > 1){
    speedup();
    fb(motorspeed);
    Serial.println("forward");
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
  Serial.println("IF 3");

}

void left(int motorspeed){

  leftmotor.writeMicroseconds(motorspeed);
  Serial.println("IF 1");

  
}

void right(int motorspeed){

  rightmotor.writeMicroseconds(motorspeed); 
  Serial.println("IF 2");  
}

void speedup(){
  if (motorspeed < motormax){
    motorspeed = motorspeed + acceleration;
    delay(100);
  }
}

void stp(){
  motorspeed = brake;
}

