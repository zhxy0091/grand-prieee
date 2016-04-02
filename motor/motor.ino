
int motorPin = 9;
void setup() {
   
 pinMode(motorPin, OUTPUT);
}
void loop() {
 analogWrite(motorPin, 255);
 delay(6000); 
 analogWrite(motorPin, 5);
 delay(6000);
 
}
