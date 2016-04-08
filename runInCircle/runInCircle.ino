
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

const int MID = 130; //MID point is 112 degree
//variables to store the servo position
int start = 0;  
int endpoint = 0;
int pos = 0;

int motorPin = 6;
int rightpoint;
int leftpoint;

void setup() {
  pinMode(motorPin, OUTPUT);
  /*leftpoint = myservo.read();
  Serial.print(leftpoint);*/
  // initialize position to mid
  myservo.attach(10);  //connect signal to pin 9
  myservo.write(MID+30);
  analogWrite(motorPin, 60);
  start = MID;
  //set range between MID+40 and MID-40
  rightpoint = start+40;
  leftpoint = start-40;
  
  
  Serial.begin(9600);
  
  
  // attaches the servo on pin 9 to the servo object
}

void loop() {
  
}

