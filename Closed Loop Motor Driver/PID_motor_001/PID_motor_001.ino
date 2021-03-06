#include "AutoPID.h"
#define maxOp 245
#define minOp 10
#define kp 1
#define ki 0.79
#define kd 0.98
long rpm, x, pret, avgver, setRpm = 50 ,diff = 1;
uint8_t pwmOut;

AutoPID pid(&avgver, &setRpm, &pwmOut, minOp, maxOp, kp, ki, kd);

volatile long time1, time2, dt;
volatile long f;
volatile uint8_t h, count;


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
}

void setup() {
  pinMode(7, OUTPUT);
  pinMode(9, 1);
  pinMode(10, 1);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(1, isr1, RISING);
  Serial.begin(115200);
  TCCR1A = 0b10100001;
  TCCR1B = 0b00000001;
  OCR1A = 0;
  OCR1B = 0;
   flashLed();

}

void loop() {
  x = dt;
  if (pret != x) {
    rpm = (60000.0 / x) * 14;
    avgver = runningAverage(rpm);
    Serial.println(avgver);
    //pret=x;
  }
  pid.run();

  OCR1A = pwmOut;
  OCR1B = 0;

}
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
}
void flashLed() {
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  delay(500);
  digitalWrite(7, LOW);
  delay(500);
  digitalWrite(7, HIGH);
  delay(500);
  digitalWrite(7, LOW);
  delay(500);
  digitalWrite(7, HIGH);
  delay(1000);
  digitalWrite(7, LOW);
  delay(200);
}
