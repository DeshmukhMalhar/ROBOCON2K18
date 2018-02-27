


#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#define trigPinO 52
#define echoPinO A13
#define trigPinZ 50
#define echoPinZ A12
#define proxi1 24
#define proxi2 22
#define proxi3 23
#define ultra_out 17
#define check_proxi 18
#define arm1 14
#define arm2 15
#define arm3 16
#define shuttle_pin 19
#define PIN 50
#define start_ultra 26
long durationO, distanceO, durationZ, distanceZ;
int current = 1, next = 1;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);
int counter = 0;
unsigned long current_millis, interval, prev;
int cytron1_pins[]={6},cytron2_pins[]={29,31,33,35,37,39,41,43};
int check_timer = 1000;
int relay_pins[] = {47, 49, 51};
LiquidCrystal_I2C lcd(0x26,20,4);

void printCytron(){
lcd.setCursor(0,0);
for(int i=0;i<1;i++){
  lcd.print(digitalRead(cytron1_pins[i]));
  Serial.println("Cytron1");
  Serial.print(digitalRead(cytron1_pins[i]));
  
}
lcd.setCursor(0,1);
for(int i=0;i<8;i++){
  lcd.print(digitalRead(cytron2_pins[i]));
  
}
 
}
void lukluk() {
  while (counter != 2) {
    for (int i = 0; i < 5; i++) {
      strip.begin();
      strip.setPixelColor(i, 255, 255, 255);
      strip.show();
    }
    delay(100);
    for (int i = 0; i < 5; i++) {
      strip.begin();
      strip.setPixelColor(i, 0, 0, 0);
      strip.show();
    }
    delay(100);
    counter++;
  }

  counter = 0;


}

void armCheck() {

  bool is = true;

  if (digitalRead(proxi1) == LOW && digitalRead(proxi2) == HIGH && digitalRead(proxi3) == HIGH) {
    prev = millis();
    is =  true;
    while ( millis() - prev < check_timer) {
      if (!(digitalRead(proxi1) == LOW && digitalRead(proxi2) == HIGH && digitalRead(proxi3) == HIGH)) {
        is = false;
        break;
      }
    }
    if (is) {
      next = 4;
      return;
    }
  }

  if (digitalRead(proxi1) == HIGH && digitalRead(proxi2) == LOW && digitalRead(proxi3) == HIGH) {
    prev = millis();
    is = true;
    while (millis() - prev < check_timer) {
      if (!(digitalRead(proxi1) == HIGH && digitalRead(proxi2) == LOW && digitalRead(proxi3) == HIGH)) {
        is = false;
        break;
      }

    }
    if (is) {
      next = 5;
      return;
    }

  }

  if (digitalRead(proxi1) == HIGH && digitalRead(proxi2) == HIGH && digitalRead(proxi3) == LOW) {
    prev = millis();
    is = true;
    while (millis() - prev < check_timer) {
      if (!(digitalRead(proxi1) == HIGH && digitalRead(proxi2) == HIGH && digitalRead(proxi3) == LOW)) {
        is = false;
        break;
      }

    }
    if (is) {
      next = 6;
      return;
    }
  }


}

void shuttle_throw() {
  lukluk();
  lukluk();
  //  digitalWrite(relay_pins[0], HIGH);
  //  delay(2000);
  //  digitalWrite(relay_pins[0], LOW);


  Serial.println("in shuttle throw");

  if ( next == 4) {
    digitalWrite(relay_pins[0], HIGH);
    delay(2000);
    digitalWrite(relay_pins[0], LOW);
  }
  else if (next == 5) {
    digitalWrite(relay_pins[1], HIGH);
    delay(2000);
    digitalWrite(relay_pins[1], LOW);
  }
  else if ( next == 6) {

    digitalWrite(relay_pins[2], HIGH);
    delay(2000);
    digitalWrite(relay_pins[2], LOW);
  }


  //  delay(1000);
}

void short_proxi() {



  if (!digitalRead(proxi1)) {
    digitalWrite(arm1, 1);

  }
  else digitalWrite(arm1, 0);

  if (!digitalRead(proxi2)) {
    digitalWrite(arm2, 1);

  }
  else digitalWrite(arm2, 0);

  if (!digitalRead(proxi3)) {
    digitalWrite(arm3, 1);

  }

  else digitalWrite(arm3, 0);
}

void ultrasonic_out() {


  durationO, distanceO;
  digitalWrite(trigPinO, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinO, HIGH);
  //  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinO, LOW);
  durationO = pulseIn(echoPinO, HIGH);
  distanceO = (durationO / 2) / 29.1;
  //  Serial.println(distanceO);
  if (distanceO >= 200 ) {
    digitalWrite(ultra_out, HIGH);
    //     Serial.println("h");
  }
  else {
    digitalWrite(ultra_out, LOW);
    //     Serial.println("l");
  }
}
//void ultrasonic_zone() {
//  durationZ, distanceZ;
//  digitalWrite(trigPinZ, LOW);  // Added this line
//  delayMicroseconds(2); // Added this line
//  digitalWrite(trigPinZ, HIGH);
//  //  delayMicroseconds(1000); - Removed this line
//  delayMicroseconds(10); // Added this line
//  digitalWrite(trigPinZ, LOW);
//  durationZ = pulseIn(echoPinZ, HIGH);
//  distanceZ = (durationZ / 2) / 29.1;
//  Serial.println("h");
//
//  Serial.println(distanceZ);
//  if (distanceZ >= 130) {
//    digitalWrite(ultra_zone, HIGH);
//  }
//  else {
//    digitalWrite(ultra_zone, LOW);
//    //         Serial.println("l");
//  }
//}


void setup() {
  // put your setup code here, to run once:
  lcd.init();
   lcd.backlight();
  pinMode(proxi1, INPUT);
  pinMode(proxi2, INPUT);
  pinMode(proxi3, INPUT);
  pinMode(arm1, OUTPUT);
  pinMode(arm2, OUTPUT);
  pinMode(arm3, OUTPUT);
  pinMode(ultra_out, OUTPUT);
  pinMode(check_proxi, INPUT);
  pinMode(trigPinO, OUTPUT);
  pinMode(echoPinO, INPUT);
  pinMode(trigPinZ, OUTPUT);
  pinMode(echoPinZ, INPUT);
  pinMode(start_ultra, INPUT);
  pinMode(shuttle_pin, INPUT);
  digitalWrite(ultra_out, LOW);

  for (int i = 0; i < 3; i++) {
    pinMode(relay_pins[i], OUTPUT);
  }

for (int i=0;i<8;i++){
  pinMode(cytron1_pins[i],INPUT);
  
}
for (int i=0;i<8;i++){
  pinMode(cytron2_pins[i],INPUT);
  
}
  Serial.begin(9600);

}

bool thrown = false;

void loop() {

printCytron();


  //  Serial.println(digitalRead(start_ultra));

  if (digitalRead(start_ultra) == HIGH) {
    ultrasonic_out();
  }

  if (digitalRead(check_proxi) == HIGH) {
    armCheck();
  }

  short_proxi();


  if (digitalRead(shuttle_pin) == HIGH) {

    if (thrown == false) {

      shuttle_throw();
      thrown = true;
    }

  }
  else {
    thrown = false;

  }

}

