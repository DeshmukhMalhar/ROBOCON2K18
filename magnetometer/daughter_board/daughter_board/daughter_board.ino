#include <Adafruit_NeoPixel.h>

#define trigPinO 52
#define echoPinO A13
#define trigPinZ 50
#define echoPinZ A12
//#define trigPinZ 48
//#define echoPinZ A15
#define arm1 24//proxi1
#define arm2 22//proxi2
#define arm3 23//proxi3
//#define proxy4 25
//#define proxy5 27

#define PIN 50
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);
unsigned long current_millis, interval, prev;
int counter;
int cytron1_pin[] = {29, 31, 33, 35, 37, 39, 41, 43},
                    cytron2_pin[] = {2, 3, 4, 5, 6, 7, 10, 11};
int read_pins[] = {18, 19}, write_pins[] = {14, 15, 16, 17};
uint8_t received_data, transmit_data;
int check_timer = 1000;
int relayPin[] = {51, 49, 47};
int throw_decider = 0;
bool current_ultrasonic = 0;
unsigned long durationO, distanceO, durationZ, distanceZ;

void Rx() {
  received_data = 0;
  for (int i = 0; i < 2; i++) {
    received_data |= digitalRead(read_pins[i]) << i;
  }
}

void Tx() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(write_pins[i], bitRead(transmit_data, i));
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

void shuttle_throw() {
  lukluk();
  lukluk();

  if (throw_decider = 1) {
    digitalWrite(relayPin[0], HIGH);
    delay(2000);
    digitalWrite(relayPin[0], LOW);

  } else if (throw_decider = 2) {
    digitalWrite(relayPin[1], HIGH);
    delay(2000);
    digitalWrite(relayPin[1], LOW);
  } else if (throw_decider = 3) {

    digitalWrite(relayPin[2], HIGH);
    delay(2000);
    digitalWrite(relayPin[2], LOW);
  }

  transmit_data = B00001000;
  Tx();
  transmit_data = B00000000;
  Tx();
  //  delay(1000);
}

void armCheck() {

  bool is = true;



  if (!digitalRead(arm2)) {
    prev = millis();
    is = true;
    while (millis() - prev < check_timer) {

      if (digitalRead(arm2)) {
        is = false;
        break;
      }

    }
    if (is) {
      transmit_data = B00000010;
      Tx();
      throw_decider = 2;
      delay(1000);
      transmit_data = 0;
      Tx();
      return;
    }

  }

  if (!digitalRead(arm1)) {
    prev = millis();
    is = true;
    while (millis() - prev < check_timer) {

      if (digitalRead(arm1)) {
        is = false;
        break;
      }
    }
    if (is) {
      transmit_data = B00000001;
      throw_decider = 1;
      Tx();
      delay(1000);
      transmit_data = 0;
      Tx();
      return;
    }
  }

  if (!digitalRead(arm3)) {
    prev = millis();
    is = true;
    while (millis() - prev < check_timer) {

      if (digitalRead(arm3)) {
        is = false;
        break;
      }

    }
    if (is) {
      transmit_data = B00000011;
      Tx();
      throw_decider = 3;
      delay(1000);
      transmit_data = 0;
      Tx();
      return;
    }
  }


}

void ultrasonic() {
  transmit_data = 0;
  Tx();
  if (current_ultrasonic == 0) {

    digitalWrite(trigPinO, LOW);  // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(trigPinO, HIGH);
    //  delayMicroseconds(1000); - Removed this line
    delayMicroseconds(10); // Added this line
    digitalWrite(trigPinO, LOW);
    durationO = pulseIn(echoPinO, HIGH);
    distanceO = (durationO / 2) / 29.1;
    //    Serial.println(distanceO);
    while (distanceO < 130) {

      digitalWrite(trigPinO, LOW);  // Added this line
      delayMicroseconds(2); // Added this line
      digitalWrite(trigPinO, HIGH);
      //  delayMicroseconds(1000); - Removed this line
      delayMicroseconds(10); // Added this line
      digitalWrite(trigPinO, LOW);
      durationO = pulseIn(echoPinO, HIGH);
      distanceO = (durationO / 2) / 29.1;
      //        //     Serial.println("h");
    }
    transmit_data = B00000110;
    Tx();
    current_ultrasonic = 1;
  } else {
    durationZ, distanceZ;
    digitalWrite(trigPinZ, LOW);  // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(trigPinZ, HIGH);
    //  delayMicroseconds(1000); - Removed this line
    delayMicroseconds(10); // Added this line
    digitalWrite(trigPinZ, LOW);
    durationZ = pulseIn(echoPinZ, HIGH);
    distanceZ = (durationZ / 2) / 29.1;
    while (distanceZ < 130) {
      durationZ, distanceZ;
      digitalWrite(trigPinZ, LOW);  // Added this line
      delayMicroseconds(2); // Added this line
      digitalWrite(trigPinZ, HIGH);
      //  delayMicroseconds(1000); - Removed this line
      delayMicroseconds(10); // Added this line
      digitalWrite(trigPinZ, LOW);
      durationZ = pulseIn(echoPinZ, HIGH);
      distanceZ = (durationZ / 2) / 29.1;
    }
    transmit_data = B00000110;
    Tx();
    current_ultrasonic = 0;
  }
  //    Serial.println(distanceZ);
}

void setup() {

  // put your setup code here, to run once:
  pinMode(arm1, INPUT);
  pinMode(arm2, INPUT);
  pinMode(arm3, INPUT);
  pinMode(trigPinO, OUTPUT);
  pinMode(echoPinO, INPUT);
  pinMode(trigPinZ, OUTPUT);
  pinMode(echoPinZ, INPUT);
  for (int i = 0; i < 2; i++) {
    pinMode(read_pins[i], INPUT);
  }

  for (int i = 0; i < 4; i++) {
    pinMode(write_pins[i],OUTPUT);
  }
  Serial.begin(9600);
transmit_data=0;
Tx();
}

void loop() {
//  while(1){
//    Serial.println(digitalRead(arm1));
//    Serial.println(digitalRead(arm2));
//    Serial.println(digitalRead(arm3));
//    
//  }
while(1){
  transmit_data=counter;
counter++;

 Tx();
 delay(1000);}
  Rx();
  Serial.println(received_data);
  if (received_data == B00000010) {
//    Serial.println("received from mother");
    armCheck();
  }

  if (received_data == B00000001) {
    shuttle_throw();
  }

  if (received_data == B00000011) {
//Serial.println(received_data);
    ultrasonic();
  }

}


