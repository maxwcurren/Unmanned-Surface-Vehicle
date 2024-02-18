//Transmitter
#include <SPI.h>              // include libraries
#include <LoRa.h>

String outgoing;              // outgoing message
 
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
 
String Mymessage = "";
 
 
void setup() {
  Serial.begin(9600);                   // initialize serial
  
  
  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
 
  Serial.println("LoRa init succeeded.");
}
 
void loop() {
  Mymessage = "Test\n";
  sendMessage(Mymessage);
  Serial.println("Sending " + Mymessage);
  Mymessage = "";
  Serial.println("Message sent successfully.");
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}
