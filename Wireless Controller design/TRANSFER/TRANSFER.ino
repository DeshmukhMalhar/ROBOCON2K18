/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
void setup() {
  radio.begin();
  radio.openWritingPipe(address); 
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
}
void loop() {
  for(int i=0;i<100;i++)
  {
  int text = i;
  radio.write(&text, sizeof(text));
  delay(50);
  }
}
