#include <AutoPID.h>

//pid settings and gains
#define OUTPUT_MIN 20
#define OUTPUT_MAX 240
#define KP 1
#define KI 0
#define KD 0
double rpm, setRPM = 300, x, pret,  pwmOUT ;
AutoPID myPID(&rpm, &setRPM, &pwmOUT, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
int8_t h;
volatile long time1, time2, dt;
void isr1() {
  if (h == 0) {
    time1 = micros();
    h = 1;
  } else {
    time2 = micros();
    h = 0;
  }
  dt = abs(time1 - time2);
}
void setup() {
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(0, isr1, RISING);
  Serial.begin(115200);
  pinMode(3, OUTPUT);
  pinMode(10, OUTPUT);
  DDRB |= (1 << PB3);
  TCCR2A = 0b10100001;
  TCCR2B = 0b00000010;


}

void loop() {
  x = dt;
  if (pret != x) {
    rpm = (60000.0 / x) * 14;

    Serial.println(rpm);
    pret = x;
  }
  OCR2A = pwmOUT;
  OCR2B = 0;
  myPID.run();
}
