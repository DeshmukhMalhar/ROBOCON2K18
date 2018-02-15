#include <Bounce2.h>
#include <TimedAction.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(22, 23); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
TimedAction t(200,sendAck);

void sendAck(){
    //tell that we are sending data
    //send some keyword
}
void setup(){
    radio.begin();
    radio.setAutoAck(1);                    // Ensure autoACK is enabled
    radio.enableAckPayload();               // Allow optional ack payloads
    radio.setRetries(0,15);                 // Smallest time between retries, max no. of retries
    radio.setPayloadSize(1);                // Here we are sending 1-byte payloads to test the call-response speed

}
void loop(){
    
    timedAction.check();
}