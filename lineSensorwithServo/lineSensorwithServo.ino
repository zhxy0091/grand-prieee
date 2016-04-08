
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

//degree limits for servo
const int MID = 125; //MID point for servo is 112 degree
const int RIGHT = MID+40;
const int LEFT = MID-40;

const int MID_POSITION = 64; //mid postion is mid index of 128 sized array

int PixelArray[128] ;            // Pixel array.

int CLK = 13;                    // Set pin as CLK.

int SI  = 12;                    // Set pin as SI.

int motorPin = 9;                // Set pin as motor

int i   =  0;                    // For pixel count.

int sensorValue = 0;             // sensor for saturation time.

int threshold = 1000;

int sensorSum = 0;
int sensorAvg = 0;
int prevPosition = MID_POSITION;
int position = MID_POSITION;
void setup() {
  
  myservo.attach(10);  //connect signal to pin 9
  // initialize position to mid
  myservo.write(MID);

  analogWrite(motorPin, 55); //write speed to motor (0~255)
  pinMode(CLK, OUTPUT);          // Set CLK as output.
  
  pinMode(SI, OUTPUT);           // Set SI as  output.
  
  Serial.begin(9600);
  
  
//8888888888888888888888888888888888888888888  
                                          //8
 digitalWrite(SI, HIGH);     //           //8
                             //           //8
 digitalWrite(CLK, HIGH);    //           //8
                             // Start.    //8
 digitalWrite(SI, LOW);      //           //8
                             //           //8
 digitalWrite(CLK, LOW);     //           //8
                                          //8
                                          //8   This clocks out indeterminate                      
                                          //8   pixel data from power up.
 for(i = 0; i < 128; i ++){               //8    
                                          //8
                                          //8
 digitalWrite(CLK, HIGH);                 //8
                                          //8                                        
 digitalWrite(CLK, LOW);                  //8
                                          //8
 }                                        //8
                                          //8
//8888888888888888888888888888888888888888888 
  
  
}


void loop() {
  
  
  sensorAvg = 0;
  sensorSum = 0; 
 //sensorValue = analogRead(A1);   // Get value for saturation time.
  
  
  
//starts pixel count
                               
 digitalWrite(SI, HIGH);       
                              
 digitalWrite(CLK, HIGH);      
                               
 digitalWrite(SI, LOW);        
                               
 digitalWrite(CLK, LOW);       
          
  
  
/*Pixel count and read
  set threshold
  value above threshold(white) is 1
  below is 0
  */                                                         
  for(i = 0; i < 128; i++){                                     
                                                            
   //  set threshold     
    int val = analogRead(A0);
    //delay(2);  
                                                              
    PixelArray[i] = doThreshold(val);
    
    sensorSum += PixelArray[i];
    sensorAvg += PixelArray[i] * i;
    
    
   // PixelArray[i] = val;                                                                                                        
    digitalWrite(CLK, HIGH);                                          
                                                                  
    digitalWrite(CLK, LOW);                                      
  }
  if(sensorSum == 0) {
    position = prevPosition;
  }
  else {
    position = (int)sensorAvg/sensorSum;
    prevPosition = position;
  }
  Serial.println(position);
  turnWheel(position);
  //delay(20);
  
//send data to computer and processing
  for(i = 0; i < 128; i ++){          
    //Serial.write(PixelArray[i]);
    
    Serial.print(PixelArray[i]);
    Serial.print(","); 
  }
  Serial.println(",");                                                              
 
  

}         // END

/*threshold the input analog signal from line sensor
  to 1(white) and 0(black)
  */
int doThreshold(int val) {
  if(val>threshold) {
      return 1;
    }
  else {
      return 0;
    }
}

void turnWheel(int position) {
  int degree = map(position, 0, 127, LEFT, RIGHT);
  Serial.println("turn degree: "+degree);
  myservo.write(degree);
}

