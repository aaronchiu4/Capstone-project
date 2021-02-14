// the following code controls the cart using the carts exact location 
// and the location of the gps of the mobile phone.
#include <Servo.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <math.h>

// create servo object to control a servo
Servo leftmotor;  
Servo rightmotor;

//set tx->pin3 rx->pin2
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


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
float heading = 0;//heading of the cart in angles
int targetangle = 0;//angle from north to the target


/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

void setup() {

   if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  leftmotor.attach(4);  // attaches the servo on pin 4 to the servo object
  rightmotor.attach(5);
  Serial.begin(115200);
  motorspeed = brake;
  temp = 0;
 
}

void loop() {

  //following used for testing  with fixed target angele only
  heading = getheading();
  targetangle = 90;

  //folloing used for actual implementation
  /*Alat = GPS.latitudeDegrees - 45;
  Along = GPS.longitudeDegrees + 75;
  Blat = 0;//read from bluetooth, target location
  Blong = 0;//read from bluetooth, target location
  Dlat = 4003017*((Blat-Alat)/360);
  Dlong = 4003017*((Blong-Along)/360);
  heading = getheading();
  targetangle = tan(Dlong/Dlat);
  */

  // Normalize to 0-360
  if (targetangle < 0)
  {
   targetangle = 360 + targetangle;
  }

  //display the values read by the parsed gps data
  /*
  Serial.print("A latitude");
  Serial.println(Alat);
  Serial.print("A longitude");
  Serial.println(Along);
  Serial.print("B latitude");
  Serial.println(Blat);
  Serial.print("B longitude");
  Serial.print(Blong);*/

  Serial.print("Compass Heading variable: ");
  Serial.println(heading);
  
  if (heading < targetangle - 2){ //turn right if heading is towards the left of the target
    speedup();
    left(motorspeed);
  }else if (heading > targetangle + 2){ //turn left if heading is towards the right of the target
    speedup();
    right(motorspeed);
  }else if (Dlat > 1){ // if heading is within two degrees of the target then go forward
    speedup();
    fb(motorspeed);
  }else{
    stp();
    fb(motorspeed);
  }
  


  Serial.print("motorspeed: ");
  Serial.println(motorspeed);

  

  delay(15);                           // waits for the servo to get there
}

//get new heading angle
float getheading(){

  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
  
  float Pi = 3.14159;
  
  // Calculate the angle of the vector y,x
  float tempheading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  
  // Normalize to 0-360
  if (tempheading < 0)
  {
   tempheading = 360 + tempheading;
  }
  
  return tempheading;
}

void fb(int motorspeed){
  
  leftmotor.writeMicroseconds(motorspeed);
  rightmotor.writeMicroseconds(motorspeed); 
  Serial.print("motorspeed: ");
  Serial.println(motorspeed); 
  Serial.println("FORWARD");

}

void left(int motorspeed){

  leftmotor.writeMicroseconds(motorspeed);
  Serial.println("TURNING RIGHT");

  
}

void right(int motorspeed){

  rightmotor.writeMicroseconds(motorspeed); 
  Serial.println("TURNING LEFT");  
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

