volatile long time1, time2, dt;
volatile uint8_t h;
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
}
long rpm, x, pret;
void loop() {
  x = dt;
  if (pret != x) {
    rpm = (60000.0 / x) * 14;
    Serial.println(rpm);
    pret = x;
  }
  
}
