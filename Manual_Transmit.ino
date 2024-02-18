//Transmitter
#include <SPI.h>              // include libraries
#include <LoRa.h>

String outgoing;              // outgoing message
 
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
 
String Mymessage = "";

int data = 0;
int manual = 0;
int throttle = 511;
int steering = 511;
int received = 0;
 
void setup() {
  Serial.begin(9600);                   // initialize serial
  
  
  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
 
  Serial.println("LoRa init succeeded.");
}
 
void loop() {
  if (Serial.available() >= 1) {
    Serial.println("Before Receiving");
    Serial.readBytes((byte*)&received, sizeof(int));
    Serial.flush();

    Serial.println("Received");

    if(received % 2 == 1) {
      throttle = received;
    }
    else if(received % 2 == 0) {
      steering = received;
    }

    String controls = String(throttle) + ',' + String(steering);
    Mymessage = controls;
  }
  
  else {
    Mymessage = "";
  }

  delay(100);
  if(Mymessage != "") {
    Serial.println("Before Transmitting");
    sendMessage(Mymessage);
    Serial.println("Sending " + Mymessage);
    Mymessage = "";
    Serial.println("Message sent successfully.");
    Serial.println("After transmitting");
  }
  else {
    Serial.println("Nothing\n");
  }
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  Serial.println("beginPacket");
  LoRa.write(destination);              // add destination address
  Serial.println("destinationAddress");
  LoRa.write(localAddress);             // add sender address
  Serial.println("senderAddress");
  LoRa.write(msgCount);                 // add message ID
  Serial.println("messageID");
  LoRa.write(outgoing.length());        // add payload length
  Serial.println("payloadLength");
  LoRa.print(outgoing);                 // add payload
  Serial.println("payload");
  LoRa.endPacket();                     // finish packet and send it
  Serial.println("FinishPacket");
  msgCount++;                           // increment message ID
}
