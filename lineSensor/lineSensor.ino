int PixelArray[128] ;            // Pixel array.

int CLK = 13;                    // Set pin as CLK.

int SI  = 12;                    // Set pin as SI.

int i   =  0;                    // For pixel count.

int sensorValue = 0;             // sensor for saturation time.

int threshold = 800;



void setup() {
  
  
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
  
  
  
 //sensorValue = analogRead(A1);   // Get value for saturation time.
  
  
  
//starts pixel count
delayMicroseconds(10000);   
 digitalWrite(SI, HIGH);       
                              
 digitalWrite(CLK, HIGH);      
                               
 digitalWrite(SI, LOW);        
                               
 digitalWrite(CLK, LOW);       
          
  
  
//Pixel count and read
//set threshold
//value above threshold(white) is 1
//below is 0
                                                                 
  for(i = 0; i < 128; i++){                                     
                                                            
   //  set threshold     
    int val = analogRead(A0);
    //delay(2);  
                                                              
    
    PixelArray[i] = analogRead(A0);                                                                                                        
    digitalWrite(CLK, HIGH);                                          
                                                                  
    digitalWrite(CLK, LOW);                                      
  }
  
  long sum=0;
//send data to computer and processing
  for(i = 0; i < 128; i ++){          
    //Serial.write(PixelArray[i]);
    
    sum += PixelArray[i];
    
  }
  Serial.println(sum/128);                                                             
  
  

}         // END

/*threshold the input analog signal from line sensor
  to 1(white) and 0(black)
  */
void doThreshold(int val) {
  if(val>threshold) {
      PixelArray[i] = 1;
    }
    else {
      PixelArray[i] = 0;
    }
}
