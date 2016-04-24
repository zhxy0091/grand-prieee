#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input = 60;
  Setpoint = 64;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  Serial.begin(9600);
}

void loop()
{
  Input = 60;
  myPID.Compute();
  Serial.println(Output);
}
