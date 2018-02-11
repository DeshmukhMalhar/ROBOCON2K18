#include "AutoPID.h"
#include <Wire.h>
#include <I2C_Anything.h> //REF: http://gammon.com.au/i2c 

#define DEBUG //comment this out for no serial output,less flash size

#define maxOp 240
#define minOp 10

#define kp 1
#define ki 0.79
#define kd 0.98

uint8_t I2C_Addr = 0x50; //last 4 bits will be generated by PINC last 4 bits

long rpm, x, pret, avgver, setRpm = 100 , diff = 1;
uint8_t pwmOut;//value of output PWM
int rpmIN;//incomeing RPM will be stored here

volatile long time1, time2, dt;
volatile long f;
volatile uint8_t h, count;

volatile boolean haveData = false;
volatile int data = 0, rpmout; //incomeing I2C data will be stored here,Only 16bit int


AutoPID pid(&avgver, &setRpm, &pwmOut, minOp, maxOp, kp, ki, kd);//PID constructor


//ISR for capturing the Encoder waveform
void isr1() {
  if (h == 0) {
    time1 = micros();
    h = 1;
  } else
  {
    time2 = micros();
    h = 0;
  }
  f = time1 - time2;
  dt = abs(f);
}//End of ISR on Hardware

void setup() {
  //interrupt pin definitations
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(1, isr1, RISING);

#ifdef DEBUG
  Serial.begin(115200);
#endif

  //Setup TIMER1 for phase correct PWM on both channels, with nonInverting PWM mode
  TCCR1A = 0b10100001;//COM1A1,COM1A2,COM1B1,COM1B2,R,R,WGM11,WGM10
  TCCR1B = 0b00000001;//
  OCR1A = 0;
  OCR1B = 0;

  //PWM pin ,OCR1x;x:A..B
  DDRB |= (1 << PB1) | (1 << PB2);

  //Init I2C
  Wire.begin(getI2Caddr());//get the address from the DIP sw on PORTC
  Wire.onReceive (receiveEvent);//attach function to the I2C interrupt

  //flash the LED and tell that we are ready to go
  flashLed();
  sei();//enable interrupts

}//END of Setup

void loop() {
  x = dt;
  if (pret != x) {
    rpm = (60000.0 / x) * 14;
    avgver = runningAverage(rpm);
    rpmout = avgver;

#ifdef DEBUG
    Serial.println(avgver);
#endif
    //pret=x;//For running changing the PID variables only on changes in RPM,!!BETA!!
  }
  pid.run();//Needs to be called once in a while in loop

  //if,data is arrived on I2C,store it in a non Volatile variable
  if (haveData) {
    rpmIN = data;
    setRpm = abs(rpmIN);
    haveData = false;
#ifdef DEBUG
    Serial.print("Got rpm:");
    Serial.println(data);
#endif
  }
  //Check for the sign of the RPM,and determine the direction of rotation
  if (rpmIN > 0) {
    OCR1A = pwmOut;
    OCR1B = 0;
  } else if (rpmIN < 0) {
    OCR1A = 0;
    OCR1B = pwmOut;
  } else {
    OCR1B = 0;
    OCR1A = 0;
  }
}//End of loop

//Running Average function for filtering out the RPM readings
long runningAverage(int M) {
#define LM_SIZE 64
  static int LM[LM_SIZE];      // LastMeasurements
  static byte index = 0;
  static long sum = 0;
  static byte count = 0;

  // keep sum updated to improve speed.
  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index++;
  index = index % LM_SIZE;
  if (count < LM_SIZE) count++;

  return sum / count;
}//END of runningAverage

void flashLed() {
#define LEDPIN 7
#define DELAY_SHORT 40
#define DELAY_LONG 100
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

//Function to read PORTC input values and determine the I2C Address
uint8_t getI2Caddr() {
  DDRC = 0;
  PORTC = 0xff;
  uint8_t pinc = 0;
  pinc = PINC;
  return ((pinc & 0x0f) + I2C_Addr);
}

//I2C receive ISR,from http://gammon.com.au/i2c
void receiveEvent (int howMany)
{
  if (howMany >= (sizeof data))
  {
    I2C_readAnything (data);
    haveData = true;
  }  // end if have enough data
}  // end of receiveEvent

//I2C request ISR
void requestEvent() {
  I2C_writeAnything(rpmout);
}

