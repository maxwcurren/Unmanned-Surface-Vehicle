int remainT1=0;
int remainT2=0;
int EscInput=511;
int ServoInput=511;
void setup() {
  Serial.begin(1200);
  pinMode(9,OUTPUT);
  pinMode(11,OUTPUT);
  digitalWrite(8, HIGH);      //Led check light
}

void loop() {
  //int timeS = millis();  // for timing setup

    if (Serial.available() >= 1) {   // for reading and determining if throttle or steering value 
      int receivedValue;
      int lastVal=0;
      Serial.readBytes((byte*)&receivedValue, sizeof(int));
      Serial.flush();
      //Serial.println(receivedValue);
      // Send the processed value back to the Raspberry Pi
      // Serial.write((byte*)&receivedValue, sizeof(int));
      // Serial.flush();
      
      //Serial.println(receivedValue);
      if(receivedValue>1024 || receivedValue<=0){
        receivedValue=lastVal;
       
      }
      
      else{ 
        if (receivedValue % 2 == 1) {
          EscInput=receivedValue;
          //Serial.print("EscInput: ");
          //Serial.print(EscInput);
          //Serial.print("\n");
          //Serial.write((char*)&EscInput, sizeof(int));
        }
        if (receivedValue % 2 == 0) {
          ServoInput=receivedValue;
          //Serial.print("ServoInput: ");
          //Serial.print(ServoInput);
          //Serial.print("\n");
        }
        lastVal=receivedValue;
      }
      // Serial.write((byte*)&receivedValue, sizeof(int));
      // Serial.flush();
    }
    
  int throttle = map(EscInput, 0, 1024, 1100, 1800); //maps input 0-1023 to output
  int steering = map(ServoInput, 0, 1024, 1000, 2000);
  
  // Ensure motors don't get out of range value
  if (throttle > 600) {throttle = 585;}
  else if (throttle < 410) {throttle = 410}
  if (steering > 1022) {steering = 1022;}
  else if (steering < 2) {steering = 2;}
  
  // 1-->2 ms delay min-->max speed esc
    digitalWrite(11, HIGH);
    delayMicroseconds(throttle);
    digitalWrite(11, LOW);
    remainT1=10000-throttle;
    delayMicroseconds(remainT1);
  //servo 
    digitalWrite(9, HIGH);
    delayMicroseconds(steering);
    digitalWrite(9, LOW);
    remainT2 = 10000 - steering;
    delayMicroseconds(remainT2);

    //int timeE=millis();
    //Serial.println(timeE-timeS); // for adjusting full loop time as computation adds time to loop
}
