#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3f, 16, 2);
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
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(1, isr1, RISING);
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.print(sizeof(long long));
  delay(1000);
  lcd.clear();

}
long rpm, x, pret;
void loop() {
  x = dt;
  if (x != pret) {
    rpm = (60000.0 / x) * 14;2
    Serial.println(rpm);
    lcd.home();
    lcd.print(rpm);
    
    pret = x;
    delay(100);
  }
}
