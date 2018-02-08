#include <Wire.h>
#include <I2C_Anything.h>
#define LED_BUILTIN 7
byte MY_ADDRESS ;

void setup()
{
  TCCR1A = 0b10100001;
  TCCR1B = 0b00000001;
  DDRB |= (1 << PB1) | (1 << PB2);
  OCR1A = 0;
  OCR1B = 0;

  MY_ADDRESS = getAddress();
  Wire.begin (MY_ADDRESS);
  Serial.begin (9600);
  Wire.onReceive (receiveEvent);
  Serial.print("HI! Address is:0x");
  Serial.println(MY_ADDRESS, HEX);
  flashLed();
}  // end of setup

volatile boolean haveData = false;
volatile int foo;

void loop()
{
  if (haveData)
  {
    Serial.print ("Received foo = ");
    Serial.println (foo);
    if (foo > 0 ) {
      OCR1A = foo;
      OCR1B = 0;
    } else if (foo < 0) {
      OCR1B = -foo;
      OCR1A = 0;
    } else {
      OCR1A = 0;
      OCR1B = 0;
    }

    haveData = false;
  }  // end if haveData

}  // end of loop

// called by interrupt service routine when incoming data arrives
void receiveEvent (int howMany)
{
  if (howMany >= (sizeof foo))
  {
    I2C_readAnything (foo);
    haveData = true;
  }  // end if have enough data
  digitalWrite(7, !digitalRead(7));
}  // end of receiveEvent

uint8_t getAddress() {
  DDRC = 0;
  PORTC = 0xff;
  uint8_t pinc = 0;
  pinc = PINC;
  return ((pinc & 0x0f) + 0x50);
}
void flashLed() {
#define LEDPIN 7
#define DELAY_SHORT 80
#define DELAY_LONG 200
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(7, HIGH);
  delay(DELAY_SHORT);
  digitalWrite(LEDPIN, LOW);
  delay(DELAY_SHORT);
  digitalWrite(LEDPIN, HIGH);
  delay(DELAY_SHORT);
  digitalWrite(LEDPIN, LOW);
  delay(DELAY_SHORT);
  digitalWrite(LEDPIN, HIGH);
  delay(DELAY_LONG);
  digitalWrite(LEDPIN, LOW);
  delay(DELAY_SHORT);
}//End of flashLed
