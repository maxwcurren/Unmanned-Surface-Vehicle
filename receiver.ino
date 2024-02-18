//Receiver
#include <SPI.h>              // include libraries
#include <LoRa.h>

String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
 
byte localAddress = 0xFF;     // address of this device
byte destination = 0xBB;      // destination to send to
 
 
 
void setup() {
  Serial.begin(9600);                   // initialize serial;
  if (!LoRa.begin(868E6)) {             
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
 
  Serial.write("LoRa init succeeded.");
}
 
void loop() {
 
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
  delay(100);
  //Serial.println("test");
}
 
 
void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length
 
  String incoming = "";
  //Serial.println("test");
 
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
 
  if (incomingLength != incoming.length()) {   // check length for error
   // Serial.println("error: message length does not match length");
   ;
    return;                             // skip rest of function
  }
 
  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    //Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  }
  //delay(98);
  Serial.println("Message: " + incoming);
}
