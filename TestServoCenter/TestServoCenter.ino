
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

const int MID = 120; //MID point is 112 degree
//variables to store the servo position
int start = 0;  
int endpoint = 0;
int pos = 0;
int rightpoint = start+20;
int leftpoint = start-20;
void setup() {
  myservo.attach(10);  //connect signal to pin 9
  /*leftpoint = myservo.read();
  Serial.print(leftpoint);*/
  // initialize position to mid
  myservo.write(MID);

  Serial.begin(9600);
  Serial.print("The start degree is: " + start);
  
  // attaches the servo on pin 9 to the servo object
}

void loop() {
  
  
}

