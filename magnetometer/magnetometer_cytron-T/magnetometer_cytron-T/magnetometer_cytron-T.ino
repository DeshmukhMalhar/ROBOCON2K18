/*
  C
  Y
  T
  R
  O
  N
*/

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
//#define interrupt_pin 2
#define arm1 14
#define arm2 15
#define arm3 16
#define ultra_out 17
#define check_proxi 18
#define shuttle_pin 19
#define start_ultra 50
#define TZ1SW 22
#define TZ2SW 25
#define TZ3SW 27
#include <Wire.h>


Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//LiquidCrystal_I2C lcd1(0x27, 20, 4), lcd2(0x26, 16, 2);
int previous_state, counter_decider;
uint8_t Kp = 12, constant = 15;
uint8_t speedo = 0;
uint8_t motor_pins[] = {11, 10, 2, 3, 5, 4, 6, 7};
uint8_t corr_velocity[] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t sensor1, sensor2;
float  minimum, maximum, variable, headingDegrees;
float heading, declinationAngle;
uint8_t velocity[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int temp_normal = 100;
int lowest = 47, low = 57, medium = 67, high = 82, normal = 124, lower_speed = 64, higher_speed = 154;

uint8_t threshold ;
//uint8_t relayPin[] = {23, 25, 27}; //23 tz1; 25 tz2 ; 27 tz3

int junction_counter = 0;
bool flag = false;
bool junction = false;

uint8_t current = 1, next = 2;
uint8_t mz1, mz2, mz3, mz4;
uint8_t mo1, mo2, mo3, mo4;
unsigned long current_millis, interval, prev;
int i;
int check_timer = 1000;
int calibration_time = 2000;
int counter;


void shuttle_throw() {

  digitalWrite(shuttle_pin, HIGH);
  delay(3000);
  digitalWrite(shuttle_pin, LOW);


}
void armCheck() {


  digitalWrite(check_proxi, HIGH);

  bool is = true;

  if (digitalRead(arm1) == HIGH && digitalRead(arm2) == LOW && digitalRead(arm3) == LOW) {
    prev = millis();
    is =  true;
    while ( millis() - prev < check_timer) {
      if (!(digitalRead(arm1) == HIGH && digitalRead(arm2) == LOW && digitalRead(arm3) == LOW )) {
        is = false;
        break;
      }
    }
    if (is) {
      next = 4;
      return;
    }
  }

  if (digitalRead(arm1) == LOW && digitalRead(arm2) == HIGH && digitalRead(arm3) == LOW) {
    prev = millis();
    is = true;
    while (millis() - prev < check_timer) {
      if (!(digitalRead(arm1) == LOW && digitalRead(arm2) == HIGH && digitalRead(arm3) == LOW)) {
        is = false;
        break;
      }

    }
    if (is) {
      next = 5;
      return;
    }

  }

  if (digitalRead(arm1) == LOW && digitalRead(arm2) == LOW && digitalRead(arm3) == HIGH) {
    prev = millis();
    is = true;
    while (millis() - prev < check_timer) {
      if (!(digitalRead(arm1) == LOW && digitalRead(arm2) == LOW && digitalRead(arm3) == HIGH)) {
        is = false;
        break;
      }

    }
    if (is) {
      next = 6;
      return;
    }
  }
  digitalWrite(check_proxi, LOW);

}




//void print_lcd() {
//
//
//
//  lcd1.setCursor(0, 0);
//  lcd1.print("Cytron1");
//  lcd1.setCursor(0, 1);
//  for (int i = 7; i >= 0; i--) {
//    lcd1.print(bitRead(PINC, i));
//  }
//  lcd1.setCursor(0, 2);
//  lcd1.print("Cytron2");
//  lcd1.setCursor(0, 3);
//  for (int i = 7; i >= 0; i--) {
//    lcd1.print(bitRead(PINL, i));
//  }
//  //  //
//  lcd1.setCursor(9, 0);
//  lcd1.print("   J=");
//  lcd1.print(junction_counter);
//}




void initt() {
  for ( i = 0; i < 8; i++) {
    pinMode(motor_pins[i], OUTPUT);
  }



}


void displaySensorDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);

}

void printSensor() {
  ////Serial.print("Sensor 1 =");
  ////Serial.println(PINC, BIN);
  ////Serial.print("Sensor 2 =");
  ////Serial.println(PINL, BIN);
}


bool is_junction_out() {
  sensor1 = PINC;
  if ((sensor1 | B10000001) == B11111111) {
    return true;
  } else return false;

}

bool is_junction_zone() {
  sensor2 = PINL;
  if ((sensor2 | B10000001) == B11111111) {
    return true;
  } else return false;

}


void set_speed_out() {


  sensors_event_t event;
  mag.getEvent(&event);

  heading = atan2(event.magnetic.y, event.magnetic.x);
  declinationAngle = 0.005;

  heading += declinationAngle;
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
  headingDegrees = heading * 180 / M_PI;
  //   //Serial.println(headingDegrees);





  if (headingDegrees < minimum) {
    //    //Serial.print("headingDegrees < minimum");
    //    //Serial.print(headingDegrees);
    //    //Serial.println("minimum");
    //    //Serial.print(minimum);
    //    //Serial.println("maximum");
    //    //Serial.print(maximum);
    speedo = (minimum - headingDegrees) * Kp + constant;
    if (speedo > threshold) {
      speedo = threshold;
    }
    //    ////Serial.println(speedo);
    corr_velocity[mo1] = speedo;

    corr_velocity[mo3] = 0;
    corr_velocity[mo4] = 0;
    //     corr_velocity[2] = speedo;
    //
    //    corr_velocity[6] = 0;
    //    corr_velocity[7] = 0;

  } else if (headingDegrees > maximum) {
    //    //Serial.println("headingDegrees > maximum");
    speedo = (headingDegrees - maximum) * Kp + constant;
    if (speedo > threshold) {
      speedo = threshold;
    }
    //    ////Serial.println(speedo);
    corr_velocity[mo3] = speedo;

    corr_velocity[mo1] = 0;
    corr_velocity[mo2] = 0;

    //       corr_velocity[6] = speedo;
    //
    //    corr_velocity[2] = 0;
    //    corr_velocity[3] = 0;

  } else {
    corr_velocity[2] = 0;
    corr_velocity[3] = 0;
    corr_velocity[6] = 0;
    corr_velocity[7] = 0;
  }
  for ( i = 0; i < 8; i++) {

    analogWrite(motor_pins[i],
                velocity[i] == 0 ? 0 : (velocity[i] == normal ? velocity[i] + corr_velocity[i] : velocity[i]));
    //    //Serial.println(velocity[i]);
  }

}

void set_speed_zone() {

  sensors_event_t event;
  mag.getEvent(&event);

  heading = atan2(event.magnetic.y, event.magnetic.x);
  declinationAngle = 0.005;

  heading += declinationAngle;
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
  headingDegrees = heading * 180 / M_PI;
  //  ////////Serial.println(headingDegrees);


  //  ////////Serial.println(headingDegrees);
  if (headingDegrees < minimum) {
    //////////Serial.println("LEft");
    speedo = (minimum - headingDegrees) * Kp + constant;
    if (speedo > threshold) {
      speedo = threshold;
    }



    corr_velocity[mz1] = speedo;

    corr_velocity[mz3] = 0;
    corr_velocity[mz4] = 0;




  } else if (headingDegrees > maximum) {
    //////////Serial.println("Right");
    speedo = (headingDegrees - maximum) * Kp + constant;
    if (speedo > threshold) {
      speedo = threshold;
    }
    corr_velocity[mz3] = speedo;

    corr_velocity[mz1] = 0;
    corr_velocity[mz2] = 0;


  } else {
    corr_velocity[1] = 0;
    corr_velocity[0] = 0;
    corr_velocity[4] = 0;
    corr_velocity[5] = 0;
  }
  for ( i = 0; i < 8; i++) {
    //    ////////Serial.print(velocity[0)
    analogWrite(motor_pins[i],
                velocity[i] == 0 ? 0 : (velocity[i] == normal ? velocity[i] + corr_velocity[i] : velocity[i]));
  }

}
//////////////////////////////////////////////////////////////////////
void angular_adj_zone() {



  sensors_event_t event;
  mag.getEvent(&event);

  heading = atan2(event.magnetic.y, event.magnetic.x);
  declinationAngle = 0.005;

  heading += declinationAngle;
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
  headingDegrees = heading * 180 / M_PI;
  //  ////////Serial.println(headingDegrees);


  //  ////////Serial.println(headingDegrees);
  if (headingDegrees < minimum) {
    //////////Serial.println("LEft");
    speedo = (minimum - headingDegrees) * Kp + 90;
    if (speedo > threshold) {
      speedo = threshold;
    }

    corr_velocity[mz1] = speedo;

    corr_velocity[mz3] = 0;
    corr_velocity[mz4] = 0;


  } else if (headingDegrees > maximum) {
    //////////Serial.println("Right");
    speedo = (headingDegrees - maximum) * Kp + 90;
    if (speedo > threshold) {
      speedo = threshold;
    }




    corr_velocity[mz3] = speedo;

    corr_velocity[mz1] = 0;
    corr_velocity[mz2] = 0;

  } else {
    corr_velocity[1] = 0;
    corr_velocity[0] = 0;
    corr_velocity[4] = 0;
    corr_velocity[5] = 0;
  }
  for ( i = 0; i < 8; i++) {
    //    ////////Serial.print(velocity[0)
    analogWrite(motor_pins[i],                velocity[i] == 0 ? 0 : corr_velocity[i]);

    //                velocity[i] == 0 ? 0 : (velocity[i] == normal ? velocity[i] + corr_velocity[i] : velocity[i]));
  }

}
///////////////////////////////////////////////////

void adj_out() {

  printSensor();
  //  print_lcd();

  sensor1 = PINC;
  //LEFT adj
  if ((sensor1 | B00011000) == B00111000) {
    velocity[0] = 0;
    velocity[1] = lowest;  //motor 1
    velocity[4] = 0;
    velocity[5] = lowest;  //motor 3
    //    //////////Serial.println("small left");
    previous_state = -1;

  }
  //    small left
  else if ((sensor1 | B00011000) == B01111000) {
    velocity[0] = 0;
    velocity[1] = low;  //motor 1
    velocity[4] = 0;
    velocity[5] = low;  //motor 3
    //    //////////Serial.println("small left");
    previous_state = -1;

  }
  //medium left
  else if ((sensor1 | B00100000) == B11100000) {
    velocity[0] = 0;
    velocity[1] = medium;  //motor1
    velocity[4] = 0;
    velocity[5] = medium;  //motor3
    //    //////////Serial.println("medium left");
    previous_state = -1;

  }
  //large left
  else if ((sensor1 | B01110000) == B11110000) {
    velocity[0] = 0;
    velocity[1] = high;  //motor 1
    velocity[4] = 0;
    velocity[5] = high;  //motor 3
    //    //////////Serial.println("large left");
    previous_state = -1;

  }
  else if ((sensor1 | B00011000) == B00011100) {
    velocity[0] = lowest; //motor 1
    velocity[1] = 0;
    velocity[4] = lowest;
    velocity[5] = 0;  //motor 3
    //    //////////Serial.println("small right");
    previous_state = 1;

  }

  //RIGHT adj
  //small right
  else if ((sensor1 | B00011000) == B00011110) {
    velocity[0] = low; //motor 1
    velocity[1] = 0;
    velocity[4] = low;
    velocity[5] = 0;  //motor 3
    //    //////////Serial.println("small right");
    previous_state = 1;

  }

  //medium right
  else if ((sensor1 | B00000100) == B00000111) {
    velocity[0] = medium; //motor 1
    velocity[1] = 0;
    velocity[4] = medium;
    velocity[5] = 0;     //motor 3
    //    //////////Serial.println("medium right");
    previous_state = 1;

  }
  //large right
  else if ((sensor1 | B00000010) == B00000011) {
    velocity[0] = high; //motor 1
    velocity[1] = 0;
    velocity[4] = high;
    velocity[5] = 0;  //motor 3
    previous_state = 1;

    //    //////////Serial.println("forward");
  } else if ((sensor1 | B00100100) == B00111100) {
    velocity[0] = 0; //motor 1
    velocity[1] = 0;
    velocity[4] = 0;
    velocity[5] = 0;
    previous_state = 0;

  }
  else if (sensor1 == 0) {
    if ((current ==  3 && next == 4) || (current ==  2 && (next == 5 || next == 6))) {

      velocity[0] = 0;
      velocity[1] = high;  //motor 1
      velocity[4] = 0;
      velocity[5] = high;

    }
    if (current == 3 && next == 3) {
      if (previous_state == 1) {
        velocity[0] = high;
        velocity[1] = 0;  //motor 1
        velocity[4] = high;
        velocity[5] = 0;

      }
      else {
        velocity[0] = 0;
        velocity[1] = high;  //motor 1
        velocity[4] = 0;
        velocity[5] = high;

      }
    }
  }

  //  if (sensor1 = 0) {
  //    if () {
  //
  //      velocity[0] = high;
  //      velocity[1] = 0;  //motor 1
  //      velocity[4] = high;
  //      velocity[5] = 0;
  //
  //    }
  //
  //  }
  if (is_junction_out()) {
    if (junction == false) {
      junction_counter++;
    }
    junction = true;
  } else {
    junction = false;
  }
  set_speed_out();
}

int zone_checker = 0;


void adj_zone() {
  //////////Serial.println("Zone");
  sensor2 = PINL;
  //  print_lcd();
  //  printSensor();
  //Forward adj

  if ((sensor2 | B00011000) == B00111000) {
    velocity[3] = lowest; //motor 2
    velocity[2] = 0;
    velocity[7] = lowest;
    velocity[6] = 0;  //motor 4
    //previous_state = -1;
    zone_checker = 1;

  }
  //small forward
  else if ((sensor2 | B00011000) == B01111000) {
    velocity[3] = low; //motor 2
    velocity[2] = 0;
    velocity[7] = low;
    velocity[6] = 0;  //motor 4
    //previous_state = -1;
    zone_checker = 1;

  }
  //medium forward
  else if ((sensor2 | B00100000) == B11100000) {
    velocity[3] = medium; //motor 2
    velocity[2] = 0;
    velocity[7] = medium;
    velocity[6] = 0;  //motor 4
    //previous_state = -1;
    zone_checker = 1;

  }
  //large forward
  else if ((sensor2 | B01000000) == B11000000) {
    velocity[3] = high; //motor 2
    velocity[2] = 0;
    velocity[7] = high;
    velocity[6] = 0;  //motor 4
    zone_checker = 1;

  }
  //BACKWARD adj
  else if ((sensor2 | B00011000) == B00011100) {
    velocity[3] = 0; //motor 2
    velocity[2] = lowest;
    velocity[7] = 0;
    velocity[6] = lowest;  //motor 4
    //previous_state = 1;

  }
  //small backward
  else if ((sensor2 | B00011000) == B00011110) {
    velocity[3] = 0; //motor 2
    velocity[2] = low;
    velocity[7] = 0;
    velocity[6] = low;  //motor 4
    //previous_state = 1;

  }
  //medium backward
  else if ((sensor2 | B00000100) == B00000111) {
    velocity[3] = 0; //motor 2
    velocity[2] = medium;
    velocity[7] = 0;
    velocity[6] = medium;  //motor 4
    //previous_state = 1;
    zone_checker = -1;

  }
  //large backward
  else if ((sensor2 | B00000010) == B00000011) {
    velocity[3] = 0; //motor 2
    velocity[2] = high;
    velocity[7] = 0;
    velocity[6] = high;  //motor 4
    //previous_state = 1;
    zone_checker = -1;


  } else if ((sensor2 | B00100100) == B00111100) {
    velocity[3] = 0; //motor 1
    velocity[2] = 0;
    velocity[6] = 0;
    velocity[7] = 0;
    zone_checker = -1;
  }
  if (sensor2 == 0) {
    //    if ((current == 1 || current == 2 ) && ( next == 4 || next == 5 || next == 6)) {
    if (zone_checker == 1) {
      velocity[3] = high; //motor 2
      velocity[2] = 0;
      velocity[7] = high;
      velocity[6] = 0;  //motor 4
    }
    else {
      velocity[3] = 0; //motor 2
      velocity[2] = high;
      velocity[7] = 0;
      velocity[6] = high;  //motor 4
    }

  }

  if (sensor2 == 0) {
    if ((current == 3) && (next == 4)) {
      velocity[3] = 0; //motor 2
      velocity[2] = high;
      velocity[7] = 0;
      velocity[6] = high;  //motor 4
    }
  }
  //  else if (sensor2 == 0 && current == 2 && (next == 4 || next == 5 || next == 6)) {
  //    velocity[3] = high; //motor 2
  //    velocity[2] = 0;
  //    velocity[7] = high;
  //    velocity[6] = 0;  //motor 4
  //  }
  //  else if (sensor2 == 0 && current == 3 && (next == 5 || next == 6)) {
  //    velocity[3] = high; //motor 2
  //    velocity[2] = 0;
  //    velocity[7] = high;
  //    velocity[6] = 0;  //motor 4
  //  }
  //  else if (sensor2 == 0 && current == 3 && next == 4) {
  //    velocity[3] = 0; //motor 2
  //    velocity[2] = high;
  //    velocity[7] = 0;
  //    velocity[6] = high;  //motor 4
  //  }
  if (is_junction_zone()) {
    if (junction == false) {
      junction_counter++;
    }
    junction = true;
    previous_state = 1;
  } else {
    junction = false;
  }

  set_speed_zone();
  //  lcd1.setCursor(0,0);
  //  lcd1.print(counter_decider);
}

void forward() {
  mo1 = 2;
  mo2 = 3;
  mo3 = 6;
  mo4 = 7;
  velocity[2] = normal;
  velocity[3] = 0;
  velocity[6] = normal;
  velocity[7] = 0;
  set_speed_out();

}


void forward_slow() {
  mo1 = 2;
  mo2 = 3;
  mo3 = 6;
  mo4 = 7;
  velocity[2] = normal;
  velocity[3] = 0;
  velocity[6] = normal;
  velocity[7] = 0;
  set_speed_out();

}
void backward() {
  mo1 = 7;
  mo2 = 6;
  mo3 = 3;
  mo4 = 2;
  velocity[2] = 0;
  velocity[3] = normal;
  velocity[6] = 0;
  velocity[7] = normal;
  set_speed_out();
}

void left() {
  //  mz1 = 1;
  //  mz3 = 5;
  //  mz2 = 0;
  //  mz4 = 4;

  mz1 = 5;
  mz3 = 1;
  mz2 = 4;
  mz4 = 0;

  velocity[0] = 0;
  velocity[1] = normal;
  velocity[4] = 0;
  velocity[5] = normal;
  set_speed_zone();
}

void right() {
  //  mz1 = 4;
  //  mz3 = 0;
  //  mz2 = 5;
  //  mz4 = 1;

  mz1 = 0;
  mz3 = 4;
  mz2 = 1;
  mz4 = 5;

  velocity[0] = normal;
  velocity[1] = 0;
  velocity[4] = normal;
  velocity[5] = 0;
  set_speed_zone();
}


void stop1() {

  velocity[0] = 255;
  velocity[1] = 255;
  velocity[2] = 255;
  velocity[3] = 255;
  velocity[4] = 255;
  velocity[5] = 255;
  velocity[6] = 255;
  velocity[7] = 255;
  for ( i = 0; i < 8; i++) {
    analogWrite(motor_pins[i], velocity[i]);
  }
}

void calibrate_magnetometer() {

  //Serial.println("function call");
  minimum = 1000;

  maximum = -1000;


  long currentmillis = millis();
  while (millis() < currentmillis + calibration_time) {

    sensors_event_t event;
    mag.getEvent(&event);

    float heading = atan2(event.magnetic.y, event.magnetic.x);
    float declinationAngle = 0.005;
    heading += declinationAngle;
    if (heading < 0)
      heading += 2 * PI;
    if (heading > 2 * PI)
      heading -= 2 * PI;
    headingDegrees = heading * 180 / M_PI;

    //Serial.print("Heading (degrees): ");
    //Serial.println(headingDegrees);
    if (headingDegrees < minimum)
      minimum = headingDegrees;
    else if (headingDegrees > maximum)
      maximum = headingDegrees;

  }
  //Serial.println("Done calibration");
}
////Seri
//void ISR() {
//  flag = !flag;
//}
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  initt();
  //  attachInterrupt(digitalPinToInterrupt(interrupt_pin),ISR,HIGH);
  //  pinMode(22, OUTPUT);
  pinMode(TZ1SW, INPUT);
  pinMode(TZ2SW, INPUT);
  pinMode(TZ3SW, INPUT);
  pinMode(arm1, INPUT);
  pinMode(arm2, INPUT);
  pinMode(arm3, INPUT);
  pinMode(ultra_out, INPUT);
  pinMode(check_proxi, OUTPUT);
  pinMode(start_ultra, OUTPUT);
  digitalWrite(start_ultra, HIGH);
  digitalWrite(check_proxi, 0);
  //  pinMode(51, INPUT);
  //  pinMode(23, OUTPUT);
  //  pinMode(25, OUTPUT);
  //  pinMode(27, OUTPUT);
  pinMode(shuttle_pin, OUTPUT);
  //Serial.begin(9600);
  ////Serial.println("HMC5883 Magnetometer Test"); ////////Serial.println("");
  //
  //  lcd1.init();
  //  lcd2.backlight();
  /* Initialise the sensor */

  if (!mag.begin()) {
    /* There was a problem detecting the HMC5883 ... check your connections */
    //Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1) {

      //      digitalWrite(relayPin, HIGH);
      //      delay(1000);
      //      digitalWrite(relayPin, LOW);
      //      delay(1000);
    }
  }
  ////Serial.println("Started");
  /* Display some basic information on this sensor */
  //  displaySensorDetails();

  calibrate_magnetometer();

  ////Serial.println("Done");

}



void loop() {
  //
  //     while(1){
  //
  //      if(digitalRead(18)){
  //        //Serial.println("high");
  //      }
  //      else {
  //        //Serial.println("low");
  //      }
  //     }

  //  while(1){
  //    //Serial.println(PINC,BIN);
  //  }
  //next=5;
  //shuttle_throw();
  //while(1);
  armCheck();
  Serial.println(next);



  if (current == 1 && next == 2) {
    //Serial.println("c1 n2");
    normal = 86;
    left();
    adj_zone();
    while (!junction) {
      adj_zone();

    }
    while (junction) {
      adj_zone();
    }
    while ((PINL | B00011100) == B11111100) {
      adj_zone();
    }
    while (PINL != 0) {
      adj_zone();
    }
    stop1();

    normal = higher_speed;
    threshold = 240 - constant - normal;
    forward();
    adj_out();
    junction_counter = 0;

    while (junction != true) {

      //    if (digitalRead(17)) {
      //        //                        stop1();
      //        normal = 50;
      //        forward_slow();
      //        threshold = 240 - constant - normal;
      //
      //      }
      adj_out();

    }
    while (junction == true) {
      adj_out();
    }
    ////Serial.println("After juntion");
    //      while(1);
    stop1();
    normal = 65;
    backward();
    adj_out();
    while (!junction) {
      adj_out();
    }
    while (junction) {
      adj_out();
    }
    stop1();
    //    adj_out();
    normal = 60;
    forward();
    adj_out();
    while (!junction) {
      adj_out();
    }
    adj_out();
    while (junction) {
      adj_out();
    }
    stop1();
    current = 2;
    armCheck();
    junction_counter = 0;
    normal = higher_speed;

  } else if (current == 2 && next == 4) {
    //////Serial.println("c1 n2");
        normal = higher_speed;
    
    threshold = 240 - constant - normal;
    adj_zone();


    junction_counter = 0;
    //    adj_zone();
    velocity[2] = 0;
    velocity[3] = 0;
    velocity[6] = 0;
    velocity[7] = 0;
    left();
    adj_zone();
    if (previous_state == -1) {
      counter_decider = 3;
    } else if (previous_state == 1) {
      counter_decider = 2;
    }
    else {
      counter_decider = 2;
    }

    junction_counter = 0;
    if (counter_decider == 2) {
      while (junction == true) {

        adj_zone();
      }
      junction_counter = 0;
    }
    //    while (junction == true) {
    //      adj_zone();
    //    }
    while (junction_counter != counter_decider - 1) {
      adj_zone();

    }
    normal = lower_speed;
    threshold = 240 - constant - normal;
    left();
    while (junction_counter != counter_decider) {
      adj_zone();

    }
    while (junction) {
      adj_zone();
    }
    adj_zone();
    stop1();
    right();
    while (!junction) {
      adj_zone();
    }

    normal = higher_speed;
    stop1();
    shuttle_throw();
    current = 4;
    next = 2;
    junction_counter = 0;

  } else if (current == 4 && next == 2) {

    digitalWrite(start_ultra, LOW);

    //Serial.println("inside current 4444444444444444 next 222222222222222");
    //    normal = higher_speed ;
    threshold = 240 - constant - normal;
    right();
    adj_zone();
    while (junction != true) {
      adj_zone();
    }
    while (junction == true) {
      adj_zone();
    }
    junction_counter = 0;
    while (junction_counter != 1) {
      //      if (digitalRead(ultra_zone)) {
      ////        stop1();
      ////        //Serial.println("ultrazone");
      ////          //Serial.println(junction_counter);
      //        normal = 55;
      //        right();
      //        threshold = 240 - constant - normal;
      //      }
      adj_zone();
    }

    normal = temp_normal;
    right();
    while (junction_counter != 2) {
      adj_zone();
    }
    while (junction) {
      adj_zone();
    }
    stop1();
    normal = lower_speed;
    left();
    adj_zone();
    while (!junction) {
      adj_zone();
    }

    stop1();
    //    while (1);
    current = 2;
    armCheck();

    //    next = 3;
    //    //Serial.println(":current");
    //    //Serial.println(current);
    //    //Serial.println("next");
    //    //Serial.println(next);
    junction_counter = 0;


  } else if (current == 2 && next == 3) {
    normal = higher_speed - 40;
    threshold = 240 - constant - normal;
    forward();
    adj_out();
    while (junction == true) {
      adj_out();
    }
    junction_counter = 0;
    while (junction != true) {
      adj_out();
    }
    current = 3;
    next = 5;
    normal = higher_speed;
    junction_counter = 0;
  }
  else if (current == 3 && next == 5) {
    normal = higher_speed;
    threshold = 240 - constant - normal;
    velocity[2] = 0;
    velocity[3] = 0;
    velocity[6] = 0;
    velocity[7] = 0;
    delay(500);
    left();
    adj_zone();
    while (junction == false) {
      adj_zone();
    }
    while (junction == true) {
      adj_zone();
    }
    junction_counter = 0;
    while (junction_counter != 1) {
      adj_zone();

    }
    normal = lower_speed;
    threshold = 240 - constant - normal;
    left();
    while (junction_counter != 2) {
      adj_zone();

    }
    stop1();
    //    while (1);
    //    delay(500);
    normal = higher_speed;
    shuttle_throw();
    current = 5;
    next = 3;
  }
  else if (current == 5 && next == 3) {
    normal = higher_speed;
    threshold = 240 - constant - normal;
    right();
    adj_zone();
    while (junction != true) {
      adj_zone();
    }
    while (junction == true) {
      adj_zone();
    }
    junction_counter = 0;
    while (junction_counter != 1) {
      adj_zone();
    }
    normal = temp_normal;
    right();
    while (junction_counter != 2) {
      adj_zone();
    }

    while (junction == true) {
      adj_zone();

    }

    stop1();
    current = 3;
    armCheck();
    //    next = 6;
  }
  else if (current == 3 && next == 6) {
    normal = higher_speed;
    threshold = 240 - constant - normal;
    velocity[2] = 0;
    velocity[3] = 0;
    velocity[6] = 0;
    velocity[7] = 0;
//    delay(500);
    left();
    adj_zone();
    while (junction == false) {
      adj_zone();
    }
    while (junction == true) {
      adj_zone();
    }
    junction_counter = 0;
    while (junction_counter != 4) {
      adj_zone();

    }
    normal = lower_speed;
    threshold = 240 - constant - normal;
    left();
    while (junction_counter != 5) {
      adj_zone();

    }
    stop1();
    //    while (1);
    //    delay(500);
    normal = higher_speed;
    shuttle_throw();
    current = 6;
    next = 3;

  }
  else if (current == 6 && next == 3) {
    normal = higher_speed;
    threshold = 240 - constant - normal;
    //Serial.println("in 666666666666666 to 333333333333333");
    right();
    adj_zone();
    while (junction != true) {
      adj_zone();
    }
    while (junction == true) {
      adj_zone();
    }
    junction_counter = 0;
    while (junction_counter != 4) {
      adj_zone();
    }
    normal = temp_normal - 20;
    right();
    while (junction_counter != 5) {
      adj_zone();
    }
    while (junction == true) {
      adj_zone();

    }
    normal = higher_speed;
    stop1();
    //    while (1);
    current = 3;
    //    next = 6;
    armCheck();

  }

  else if (current == 2 && next == 5) {

    //    //Serial.println("inside current 222222222222222222 next 555555555555555555");
    normal = higher_speed - 40;
    threshold = 240 - constant - normal;
    forward();
    adj_out();
    //    while (junction == true) {
    //      adj_out();
    //    }

    junction_counter = 0;
    while (junction != true) {
      adj_out();
    }
    while (junction == true) {
      adj_out();
    }
    normal = higher_speed;
    threshold = 240 - constant - normal;
    junction_counter = 0;

    velocity[2] = 0;
    velocity[3] = 0;
    velocity[6] = 0;
    velocity[7] = 0;
    left();
    adj_zone();


    if (previous_state == -1) {
      counter_decider = 3;
    } else if (previous_state == 1) {
      counter_decider = 2;
    }
    else {
      counter_decider = 2;
    }

    junction_counter = 0;
    if (counter_decider == 2) {
      while (junction == true) {

        adj_zone();
      }
      junction_counter = 0;
    }
    //    while (junction == true) {
    //      adj_zone();
    //    }
    while (junction_counter != counter_decider - 1) {
      adj_zone();

    }
    normal = lower_speed;
    threshold = 240 - constant - normal;
    left();
    while (junction_counter != counter_decider) {
      adj_zone();

    }

    normal = lower_speed;
    adj_zone();
    while (junction) {
      adj_zone();
    }
    adj_zone();
    right();
    while (!junction) {
      adj_zone();
    }

    normal = higher_speed;
    stop1();
    shuttle_throw();
    current = 5;
    next = 3;
    junction_counter = 0;
  }
  else if (current == 2 && next == 6) {
    //    //Serial.println("inside current 222222222222222222 next 555555555555555555");
    normal = higher_speed - 40;
    threshold = 240 - constant - normal;
    forward();
    adj_out();
    //    while (junction == true) {
    //      adj_out();
    //    }

    junction_counter = 0;
    while (junction != true) {
      adj_out();
    }
    while (junction == true) {
      adj_out();
    }
    normal = higher_speed;
    threshold = 240 - constant - normal;
    junction_counter = 0;

    velocity[2] = 0;
    velocity[3] = 0;
    velocity[6] = 0;
    velocity[7] = 0;
    left();
    adj_zone();


    if (previous_state == -1) {
      counter_decider = 6;
    } else if (previous_state == 1) {
      counter_decider = 5;
    }
    else {
      counter_decider = 5;
    }

    junction_counter = 0;
    if (counter_decider == 5) {
      while (junction == true) {

        adj_zone();
      }
      junction_counter = 0;
    }
    //    while (junction == true) {
    //      adj_zone();
    //    }
    while (junction_counter != counter_decider - 1) {
      adj_zone();

    }
    normal = lower_speed;
    threshold = 240 - constant - normal;
    left();
    while (junction_counter != counter_decider) {
      adj_zone();

    }

    while (junction) {
      adj_zone();
    }
    adj_zone();
    right();
    while (!junction) {
      adj_zone();
    }

    normal = higher_speed;
    stop1();
    shuttle_throw();
    current = 6;
    next = 3;
    junction_counter = 0;
  }
  else if (current == 3 && next == 4) {
    //    //Serial.println("inside 3333333333333333333 to 4444444444444444444444");
    normal = higher_speed - 30 ;
    threshold = 240 - constant - normal;
    backward();
    adj_out();
    while (!junction) {
      adj_out();
    }
    while (junction) {
      adj_out();
    }

    junction_counter = 0;
    while (junction != true) {
      adj_out();
    }
    while (junction == true) {
      adj_out();
    }

    stop1();
    junction_counter = 0;
    adj_zone();
    normal = higher_speed;
    threshold = 240 - constant - normal;
    //
    //    junction_counter = 0;

    velocity[2] = 0;
    velocity[3] = 0;
    velocity[6] = 0;
    velocity[7] = 0;
    adj_zone();
    if (previous_state == -1) {
      counter_decider = 3;
    } else if (previous_state == 1) {
      counter_decider = 2;
    }
    else {
      counter_decider = 2;
    }

    left();
    adj_zone();
    junction_counter = 0;
    if (counter_decider == 2) {
      while (junction == true) {

        adj_zone();
      }
      junction_counter = 0;
    }

    while (junction_counter != counter_decider - 1) {
      adj_zone();
    }
    stop1();
    normal = lower_speed;
    threshold = 240 - constant - normal;
    left();

    while (junction_counter != counter_decider) {
      adj_zone();

    }
    stop1();
    //    while (1);

    shuttle_throw();
    normal = higher_speed;

    current = 4;
    next = 2;
    junction_counter = 0;
  }
  else if (current == 1 && next == 4) {
    digitalWrite(start_ultra, HIGH);


    normal = higher_speed;
    threshold = 240 - constant - normal;
    //Serial.println("inside current 1111111 and next 444444444");
    forward();
    adj_out();

    while (junction != true) {

      if (digitalRead(ultra_out)) {
        //
        //        stop1();

        normal = 55;
        forward_slow();
        threshold = 240 - constant - normal;
        digitalWrite(start_ultra, LOW);
      }
      adj_out();

    }
    digitalWrite(start_ultra, LOW);
    while (junction == true) {
      adj_out();
    }
    stop1();

    junction_counter = 0;
    adj_zone();

    //
    //    junction_counter = 0;

    velocity[2] = 0;
    velocity[3] = 0;
    velocity[6] = 0;
    velocity[7] = 0;
    adj_zone();
    if (previous_state == -1) {
      counter_decider = 3;
    } else if (previous_state == 1) {
      counter_decider = 2;
    }
    else {
      counter_decider = 2;
    }
    normal = higher_speed;
    threshold = 240 - constant - normal;
    left();
    adj_zone();
    junction_counter = 0;
    if (counter_decider == 2) {
      while (junction == true) {

        adj_zone();
      }
      junction_counter = 0;
    }

    while (junction_counter != counter_decider - 1) {
      adj_zone();
    }
    stop1();
    normal = lower_speed;
    threshold = 240 - constant - normal;
    //    Kp=Kp*2;
    left();

    while (junction_counter != counter_decider) {
      adj_zone();

    }
    stop1();
    //    Kp=Kp/2;
    shuttle_throw();
    normal = higher_speed;

    current = 4;
    next = 2;
    //    //Serial.println("current");
    //    //Serial.println(current);
    //    //Serial.println("next");
    //    //Serial.println(next);
    junction_counter = 0;

  }


  else if (current == 1 && next == 5 ) {
    junction_counter = 0;
    normal = higher_speed;
    threshold = 240 - constant - normal;
    forward();
    adj_out();

    while (junction != true) {
      adj_out();

    }
    while (junction == true) {
      adj_out();

    }
    normal = temp_normal;
    threshold = 240 - constant - normal;
    forward();
    while (junction != true) {
      adj_out();

    }
    while (junction == true) {
      adj_out();

    }

    //    while (junction_counter < 1) {
    //      adj_out();
    //    }
    //    normal = temp_normal;
    //    threshold = 240 - constant - normal;
    //    //    junction_counter = 0;
    //    adj_zone();
    //    forward();
    //    while (junction_counter < 2) {
    //      adj_out();
    //    }
    stop1();


    //
    //    junction_counter = 0;

    velocity[2] = 0;
    velocity[3] = 0;
    velocity[6] = 0;
    velocity[7] = 0;
    adj_zone();
    if (previous_state == -1) {
      counter_decider = 3;
    } else if (previous_state == 1) {
      counter_decider = 2;
    }
    else {
      counter_decider = 2;
    }
    temp_normal = normal;
    left();
    adj_zone();
    junction_counter = 0;
    if (counter_decider == 2) {
      while (junction == true) {

        adj_zone();
      }
      junction_counter = 0;
    }
    while (junction != true) {
      adj_zone();
    }
    while (junction_counter != counter_decider - 1) {
      adj_zone();
    }
    //    stop1();
    normal = lower_speed;
    threshold = 240 - constant - normal;
    left();

    while (junction_counter != counter_decider) {
      adj_zone();

    }
    adj_zone();
    normal = lower_speed;
    while (junction) {
      adj_zone();
    }
    adj_zone();
    right();
    while (!junction) {
      adj_zone();
    }
    stop1();
    shuttle_throw();
    normal = higher_speed;
    current = 5;
    next = 3;

  }
  else if (current == 1 && next == 6) {
    junction_counter = 0;
    normal = higher_speed;
    threshold = 240 - constant - normal;
    forward();
    adj_out();

    while (junction != true) {
      adj_out();

    }
    while (junction == true) {
      adj_out();

    }
    normal = temp_normal;
    threshold = 240 - constant - normal;
    forward();
    while (junction != true) {
      adj_out();

    }
    while (junction == true) {
      adj_out();

    }

    //    while (junction_counter < 1) {
    //      adj_out();
    //    }
    //    normal = temp_normal;
    //    threshold = 240 - constant - normal;
    //    //    junction_counter = 0;
    //    adj_zone();
    //    forward();
    //    while (junction_counter < 2) {
    //      adj_out();
    //    }
    stop1();


    //
    //    junction_counter = 0;

    velocity[2] = 0;
    velocity[3] = 0;
    velocity[6] = 0;
    velocity[7] = 0;
    adj_zone();
    if (previous_state == -1) {
      counter_decider = 6;
    } else if (previous_state == 1) {
      counter_decider = 5;
    }
    else {
      counter_decider = 5;
    }
    temp_normal = normal;
    left();
    adj_zone();
    junction_counter = 0;
    if (counter_decider == 5) {
      while (junction == true) {

        adj_zone();
      }
      junction_counter = 0;
    }
    while (junction != true) {
      adj_zone();
    }
    while (junction_counter != counter_decider - 1) {
      adj_zone();
    }
    //    stop1();
    normal = lower_speed;
    threshold = 240 - constant - normal;
    left();

    while (junction_counter != counter_decider) {
      adj_zone();

    }
    while (junction) {
      adj_zone();
    }
    adj_zone();
    right();

    while (!junction) {
      adj_zone();
    }
    stop1();
    shuttle_throw();
    normal = higher_speed;
    current = 6;
    next = 3;

  }



  else if (current == 3 && next == 3) {
    //    delay(2000);
    normal = temp_normal;
    threshold = 240 - constant - normal;
    left();
    adj_zone();
    while (!junction) {
      adj_zone();
    }
    junction_counter = 0;
    backward();
    previous_state = 1;
    adj_out();
    while (junction_counter != 2) {
      adj_out();
    }
    //    while (junction == true) {
    //      adj_out();
    //    }
    //    while (junction != true) {
    //      adj_out();
    //    }
    //    while (junction == true) {
    //      adj_out();
    //    }
    while (junction) {
      adj_out();
    }
    stop1();
    forward();
    adj_out();
    while (!junction) {
      adj_out();
    }
    stop1();
    current = 2;
    next = 2;
    armCheck();
  }

  else if ((current == 6 && next == 6) || (current == 5 && next == 5)) {
    next = 3;


  }
  else if (current == 4 && next == 4) {
    next = 2;
  }
}








