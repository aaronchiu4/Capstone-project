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
#define GPSECHO  false

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
float Alat = 0;// latitude of point A
float Along = 0;//longitude of point A
float Blat = 0;//latitude of point B
float Blong = 0;//longitude of point B 
float Dlat = 0;//delta latitude
float Dlong = 0;//delta longitude
float heading = 0;//heading of the cart in angles
float targetangle = 0;//angle from north to the target


/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

void setup() {

   if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  setupGPS();
  
  leftmotor.attach(4);  // attaches the servo on pin 4 to the servo object
  rightmotor.attach(5);
  Serial.begin(115200);
  motorspeed = brake;
  temp = 0;
 
}

void setupGPS(){
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}
//for GPS initilization
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
uint32_t timer = millis();

void loop() {

  //following used for testing  with fixed target angele only
  heading = getheading();
  targetangle = 90;
  parseGPS();
  Alat = GPS.latitudeDegrees - 45;
  Along = GPS.longitudeDegrees + 75;

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
  Serial.print("Alat: ");
  Serial.println(Alat);
  Serial.print("Along: ");
  Serial.println(Along);
  
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

//parse GPS data
void parseGPS(){
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 15) { 
    timer = millis(); // reset the timer

    /*Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); */

    Serial.print("FIX: ");
    Serial.println(GPS.fix);
    
    if (GPS.fix) {
      
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 8);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 8);
      
     
    }
  }
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
  Serial.println("STOP");
}

