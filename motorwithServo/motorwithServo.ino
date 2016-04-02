
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

const int MID = 130; //MID point is 112 degree
//variables to store the servo position
int start = 0;  
int endpoint = 0;
int pos = 0;

int motorPin = 9;
int rightpoint;
int leftpoint;

void setup() {
  myservo.attach(10);  //connect signal to pin 9
  pinMode(motorPin, OUTPUT);
  /*leftpoint = myservo.read();
  Serial.print(leftpoint);*/
  // initialize position to mid
  myservo.write(MID);
  analogWrite(motorPin, 55);
  start = MID;
  //set range between MID+40 and MID-40
  rightpoint = start+40;
  leftpoint = start-40;
  
  
  Serial.begin(9600);
  Serial.print("The start degree is: " + start);
  
  // attaches the servo on pin 9 to the servo object
}

void loop() {
  for (pos=start; pos <= rightpoint; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);   // tell servo to go to position in variable 'pos'
    Serial.print(pos);
    Serial.print(",");
    delay(20);                       // waits 15ms for the servo to reach the position
  }
  Serial.println();
  Serial.println("Turning left: ");
  for (pos=rightpoint; pos >= leftpoint; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);
    Serial.print(pos);
    Serial.print(",");// tell servo to go to position in variable 'pos'
    delay(20);                       // waits 15ms for the servo to reach the position
  }
  start = pos;
  Serial.println();
  Serial.println("Turning right");
//  int position = (int)sensorAvg/sensorSum;
}

