
#include <Servo.h>
int EscInput=511;
int ServoInput=511;
Servo servo; 
Servo esc; 

void setup() {
  Serial.begin(1200);
  servo.attach(9); 
  esc.attach(11);
  digitalWrite(8, HIGH);      //Led check light
}

void loop() {
  //int timeS = millis();  // for timing setup

    if (Serial.available() >= 1) {   // for reading and determining if throttle or steering value 
      int receivedValue;
      int lastVal=0;
      Serial.readBytes((byte*)&receivedValue, sizeof(int));
      Serial.flush();
      // Serial.println(receivedValue);
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
          // Serial.print("EscInput: ");
          // Serial.print(EscInput);
          // Serial.print("\n");
          //Serial.write((char*)&EscInput, sizeof(int));
        }
        if (receivedValue % 2 == 0) {
          ServoInput=receivedValue;
          // Serial.print("ServoInput: ");
          // Serial.print(ServoInput);
          // Serial.print("\n");
        }
        lastVal=receivedValue;
      }
      // Serial.write((byte*)&receivedValue, sizeof(int));
      // Serial.flush();
    }
    
  int throttle = map(EscInput, 0, 1024, 60, 125); //maps input 0-1023 to output
  int steering = map(ServoInput, 1024, 0, 10, 170);

  //Serial.print("Thrott: ");
  //Serial.print(throttle);
  //Serial.print("\n");
  //Serial.print("Steer: ");
  //Serial.print(steering);
  //Serial.print("\n");
  esc.write(throttle);
  servo.write(steering);
}
