#include <Servo.h>

Servo myservo;  // create servo object to control a servo

//degree limits for servo
const int SERVO_MID = 120; //MID point for servo
const int SERVO_RIGHT = SERVO_MID+40;
const int SERVO_LEFT = SERVO_MID-40;

//Lock mechanism for servo
boolean leftLock = false;
boolean rightLock = false;

//position of cars
const int MID_POSITION = 64; //mid postion is mid index of 128 sized array
int prevPosition = MID_POSITION;
int curPosition = MID_POSITION;

//motor duty range: 0~100
const int MAX_DUTY = 40;
const int MIN_DUTY = 30;

int pixelArray[128] ;            // Pixel array.

int CLK = 13;                    // Set pin 13 as CLK.
int SI  = 12;                    // Set pin 12 as SI.
int MOTOR_PIN = 6;                // Set pin 9 as motor
int SERVO_PIN =10;                //Set pin 10 as servo

void setup() {
  leftLock = false;
  rightLock = false;
  
  pinMode(CLK, OUTPUT);          // Set CLK as output.
  pinMode(SI, OUTPUT);           // Set SI as  output.
  pinMode(MOTOR_PIN, OUTPUT);    // Set MOTOR_PIN as output.
  
  myservo.attach(SERVO_PIN);  //connect signal to pin 9
  myservo.write(SERVO_MID);   //initialize position to mid


  Serial.begin(9600);
  
  digitalWrite(SI, HIGH);    
  digitalWrite(CLK, HIGH);  
  digitalWrite(SI, LOW);      
  digitalWrite(CLK, LOW);    

  // This clocks out indeterminate pixel data from power up.
  for(int i = 0; i < 128; i ++){                                                                                                
    digitalWrite(CLK, HIGH);                                                                                
    digitalWrite(CLK, LOW);                                                       
  }                                    

  
}


void loop() {
  //wait for expose
  delayMicroseconds(10000);
  //starts pixel count
  digitalWrite(SI, HIGH);       
  digitalWrite(CLK, HIGH);      
  digitalWrite(SI, LOW);        
  digitalWrite(CLK, LOW);       
  
  //read data from camera to pixelArray                                           
  for(int i = 0; i < 128; i++){                                 
    pixelArray[i] = analogRead(A0);                                                                                                
    digitalWrite(CLK, HIGH);                                       
    digitalWrite(CLK, LOW);                                      
  }
  /*
  //print pixelArray 
  for(int i = 0; i < 128; i ++){          
    //Serial.write(PixelArray[i]);
    
    Serial.print(pixelArray[i]);
    Serial.print(","); 
  }
  Serial.println(",");   
  //end print
  */
  findPosition(pixelArray);
  Serial.println(curPosition);
  //Lock mechanism to deal with case if totally off the track
  Serial.println(pixelArray[curPosition]);
  /*
  if(105<curPosition && curPosition<112 && rightLock == true) {
    rightLock = false;
  }
  else if(20<curPosition && curPosition<30 && leftLock == true) {
    leftLock = false;
  }
  if(curPosition > 113) {
    rightLock = true;
  }
  else if(curPosition < 15) {
    leftLock = true;
  }
  */
  if(pixelArray[curPosition] > 200  && rightLock == true) {
    rightLock = false;
  }
  else if(pixelArray[curPosition] > 200 && leftLock == true) {
    leftLock = false;
  }
  if(curPosition > 95) {
    rightLock = true;
  }
  else if(curPosition < 35) {
    leftLock = true;
  }
  int degree = 0;
  if(rightLock) {
    degree = SERVO_RIGHT;
  }
  else if(leftLock) {
    degree = SERVO_LEFT;
  }
  else {
    degree = map(curPosition, 35, 95, SERVO_LEFT, SERVO_RIGHT);
  }
  turnWheel(degree);
  //offValue is how much the car is off the center
  int offValue = curPosition-MID_POSITION;
  if(offValue<0) {
     offValue = -offValue;
  }
  int motorDuty = map(offValue, 0, 64, MAX_DUTY, MIN_DUTY); //dynamically adjust the speed
  int speed = motorPWM(motorDuty);
  analogWrite(MOTOR_PIN, speed);

  


}       

// Find the position of the car
void findPosition(int* pixelArray) {
   int sensorSum = 0;
   int sensorAvg = 0;
   int threshold = findThreshold(pixelArray);
   //Serial.println(threshold);
   for(int i = 0; i < 128; i++) { 
      int afterThreshold = doThreshold(pixelArray[i], threshold);
      sensorSum += afterThreshold;
      sensorAvg += afterThreshold * i;
   }
   //Serial.println(sensorSum);
   curPosition = (int)sensorAvg/sensorSum;
   
   prevPosition = curPosition;
  
}

/* Steer wheel by certain degree based on the position of the car
 */
void turnWheel(int degree) {
  myservo.write(degree);
}

/* Calculate the duty cycle for the motor
 */
int motorPWM(int motorDuty) {
  int speed = 2.55 * motorDuty;      //speed range: 0~255
  return speed;
}

/*threshold the input analog signal from line sensor
  to 1(white) and 0(black)
 */
int doThreshold(int val, int threshold) {
  //int threshold = findThreshold(pixelArray);
  if(val>threshold) {
    return 1;
  }
  else {
    return 0;
  }
}

/* Find the best threshold to determine the location of the line
 */
int findThreshold(int* pixelArray) {
  long average = 0;
  int maximum = 0;

  for(int i = 0; i < 128; i++) {
    average = average + pixelArray[i];
    if(pixelArray[i] > maximum) {
      maximum = pixelArray[i];
    }
  }
  //Serial.println(average);
  average = average/128;
  
  //Adaptive threshold
  int offset = adaptThresOffset(maximum, average);
  
  offset = offset + average;

  //Static threshold (THRES_OFFSET varies)
  //offset = 1000;

  return offset;
}


/* Algorithm for adaptive thresholding
 */
int adaptThresOffset(int maximum, int average)
{
  return ((maximum - average)/2);
}

