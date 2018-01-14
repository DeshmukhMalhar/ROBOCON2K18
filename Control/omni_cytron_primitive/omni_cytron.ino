//#define DEBUG_S
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
int junc_counter = 0;
LiquidCrystal_I2C lcd(0x3F, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

//R from cytron stored in this
int R[] = { -10, -10, -10, -10, -10, -10, -10, -10};

//Speed of each wheel
int speed_wheel[] = {0, 0, 0, 0, 0, 0, 0, 0};

//To Mosfet - Each cuople for 1 motor (1,0-Forward)(0,1-Backward)
int motor_pins[] = {2, 5, 6, 7, 8, 9, 10, 11};

//Sensor pins from Cytron
int cytron_pins[] = {30, 31, 32, 33, 34, 35, 36, 37};

//int junction_counter  = 0;

int lowest = 40, lower = 45, low = 55, normal = 65, high = 90, randomm_lower = 43, randomm_low = 53, randomm = 61, randomm_lowest = 38;

//Initialization of pins
void initialization() {
  for (int i = 0; i < 8; i++) {
    pinMode(motor_pins[i], OUTPUT);
    digitalWrite(motor_pins[i], LOW);
    pinMode(cytron_pins[i], INPUT);

  }
}
void printSecondRow(char str[]) {
  lcd.setCursor(0, 1);
  for (int i = 0; str[i]; i++) {
    lcd.print(str[i]);
  }
}

void swap_motors() {
  int temp[] = {11, 10, 2, 5, 7, 6, 8, 9};
  for (int i = 0; i < 8; i++) {
    motor_pins[i] = temp[i];
  }
}

void swap_cytron() {
  int temp[] = {37, 36, 35, 34, 33, 32, 31, 30};
  for (int i = 0; i < 8; i++) {
    cytron_pins[i] = temp[i];
  }
}

int white = HIGH, black = LOW;
void set_speed() {
  for (int i = 0; i < 8; i++) {
    //    Serial.println("");
    //    Serial.print(motor_pins[i]);
    //    Serial.print(" ");
    //    Serial.println(speed_wheel[i]);

    analogWrite(motor_pins[i], speed_wheel[i]);
    //    delay(500);
  }

}
void assign_speed(int motor1CW, int motor1CCW, int motor2CW, int motor2CCW, int motor3CW, int motor3CCW , int motor4CW , int motor4CCW) {
  int temp[] = {motor1CW, motor1CCW, motor2CW, motor2CCW, motor3CW, motor3CCW, motor4CW, motor4CCW};
  for (int i = 0; i < 8; i++) {
    speed_wheel[i] = temp[i];
  }
}
void print_lcd() {

  lcd.setCursor(0, 0);
  for (int i = 0; i < 8; i++) {
    lcd.print(R[i]);

  }



}
void setup() {
  initialization();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("ROBOCON 2K18");
  delay(1000);
  lcd.clear();
  Serial.begin(9600);

}


void printCytron() {
  for (int i = 0; i < 8; i++) {
    Serial.print(R[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}


//Read from Cytron
void read_cytron() {
  for (int i = 0; i < 8; i++) {
    R[i] = digitalRead(cytron_pins[i]);
  }
  //  printCytron();
}

/*
   -3:lefttt
   -2:leftt
   -1:left
   0:centre
   1:right
   2:rightt
   3:righttt
   -100:left junc
   100:right junc
   1000:all white
   10000:all black
*/
int position_sensor() {
  //10111000

  if (R[0] == white && (R[2] == white || R[3] == white || R[4] == white)) {
    return 2000;
  }
  if ((R[0] == white && R[1] == black && R[2] == black && R[3] == black && R[4] == black && R[5] == black && R[6] == black && R[7] == black) ||
      (R[1] == white && R[0] == white && R[2] == black && R[3] == black && R[4] == black && R[5] == black && R[6] == black && R[7] == black) ||
      (R[0] == white && R[1] == white && R[2] == white && R[3] == black && R[4] == black && R[5] == black && R[6] == black && R[7] == black)) {

    //Serial.println("im in -3");

    return -3;

  }
  if ((R[1] == white && R[2] == white && R[0] == black && R[3] == black && R[4] == black && R[5] == black && R[6] == black && R[7] == black) ||
      (R[1] == white && R[2] == white && R[3] == white && R[0] == black && R[4] == black && R[5] == black && R[6] == black && R[7] == black)) {
    //    Serial.println("im in -2");
    return -2;
  }
  if ((R[2] == white && R[3] == white && R[0] == black && R[1] == black  && R[4] == black && R[5] == black && R[6] == black && R[7] == black) ||
      (R[2] == white && R[4] == white && R[3] == white && R[0] == black && R[1] == black && R[5] == black && R[6] == black && R[7] == black)) {
    //    Serial.println("im in -1");
    return 0;

  }
  if (R[0] == black && R[1] == black && R[2] == black && R[3] == white && R[4] == white && R[5] == black && R[6] == black && R[7] == black) {
    //    Serial.println("im in 0");
    return 0;
  }
  if ((R[5] == white && R[4] == white && R[0] == black && R[1] == black && R[2] == black && R[3] == black && R[6] == black && R[7] == black) ||
      (R[5] == white && R[4] == white && R[3] == white && R[0] == black && R[1] == black && R[2] == black && R[6] == black && R[7] == black)) {
    //    Serial.println("im in 1");
    return 0;

    //
  }
  if ((R[6] == white && R[5] == white && R[0] == black && R[1] == black && R[2] == black && R[3] == black && R[4] == black && R[7] == black) ||
      (R[6] == white && R[5] == white && R[4] == white && R[0] == black && R[1] == black && R[2] == black && R[3] == black && R[7] == black)) {
    //    Serial.println("im in 2");
    return 2;
  }
  if ((R[7] == white && R[0] == black && R[1] == black && R[2] == black && R[3] == black && R[4] == black && R[5] == black && R[6] == black ) ||
      (R[7] == white && R[6] == white && R[0] == black && R[1] == black && R[2] == black && R[3] == black && R[4] == black && R[5] == black ) ||
      (R[5] == white && R[6] == white && R[7] == white && R[0] == black && R[1] == black && R[2] == black && R[3] == black && R[4] == black )) {
    //    Serial.println("im in 3");
    return 3;//righttt
  }


  if (R[0] == white && R[1] == white && R[2] == white && R[3] == white && R[4] == white && R[5] == white && R[6] == white && R[7] == white)
  {
    return 1000;//all white
  }
  if (R[0] == black && R[1] == black && R[2] == black && R[3] == black && R[4] == black && R[5] == black && R[6] == black && R[7] == black)
  {
    return 10000;//all black
  }



  //  if ((R[0] == white && R[1] == white && R[2] == black && R[3] == white && R[4] == white && R[5] == black && R[6] == black && R[7] == black) ||
  //      (R[0] == white && R[1] == white && R[2] == white && R[3] == white && R[4] == white && R[5] == black && R[6] == black && R[7] == black) ||
  //      (R[0] == white && R[1] == white && R[2] == black && R[3] == white && R[4] == white && R[5] == white && R[6] == black && R[7] == black))
  //  {
  //    return 2000;//junction
  //  }
  /*
  (cytron_read == 11011000 ) || (cytron_read == 11111000 ) || (cytron_read == 11011100)
  */

  if (R[4] == black && R[5] == black && R[6] == black && R[7] == black)
  {
    return 3000;
  }
}
//void loop(){
//  read_cytron();
//  print_lcd();
//  }
//void loop() {
//  lcd.print("1");
//
//}
void loop() {

  read_cytron();
  print_lcd();
  set_speed();
  if (position_sensor() == 0 ) {
    if (junc_counter == 1) {
      assign_speed(0, 0, normal, 0, 0, 0, normal, 0);

    }
    else {
      assign_speed(0, 0, randomm, 0, 0, 0, normal, 0);
    }
    set_speed();

    printSecondRow("forward");
    Serial.println("0");


  }
  else if (position_sensor() == -1) {
    printSecondRow("left   ");
    if (junc_counter == 1) {
      assign_speed(lowest, 0, normal, 0,  randomm_lowest, 0, normal, 0);
    } else {


      assign_speed(lowest, 0, randomm, 0, lowest, 0, normal, 0);
    }
    Serial.println("-1");

  }
  else if (position_sensor() == -2) {
    printSecondRow("leftt  ");
    if (junc_counter == 1) {
      assign_speed(lower, 0, normal , 0, randomm_lower, 0, normal, 0);
    } else {
      assign_speed(lower, 0, randomm, 0, lower, 0, normal, 0);
    }

    Serial.println("-2");

  }

  else if (position_sensor() == -3) {
    printSecondRow("lefttt ");
    if (junc_counter == 1) {
      assign_speed(low, 0, normal , 0, randomm_low, 0, normal, 0);
    }
    assign_speed(low, 0, randomm, 0, low, 0, normal, 0);
    Serial.println("-3");
  }

  else if (position_sensor() == 1) {
    printSecondRow("right  ");
    if (junc_counter == 1) {
      assign_speed(0, lowest, normal, 0, 0, randomm_lowest, normal, 0);

    } else {
      assign_speed(0, lowest, randomm, 0, 0, lowest, normal, 0);
    }
    Serial.println("1");

  }
  else if (position_sensor() == 2) {
    printSecondRow("rightt ");
    if (junc_counter == 1) {
      assign_speed(0, lower, normal , 0, 0, randomm_lower, normal, 0);

    } else {
      assign_speed(0, lower, randomm, 0, 0, lower, normal, 0);
    }
    Serial.println("2");
  }
  else if (position_sensor() == 3) {
    printSecondRow("righttt");
    if (junc_counter == 1) {
      assign_speed(0, low, normal , 0, 0, randomm_low, normal, 0);

    } else {
      assign_speed(0, low, randomm, 0, 0, low, normal, 0);
    }
    Serial.println("3");
  }


  //  else if (position_sensor() == 1000) {
  //    printSecondRow("junctio");
  //    junction_counter = 1;
  //    assign_speed(high, 0, high, 0, 0, high, 0, high);
  //    set_speed();
  //    //    j++;
  //
  //    while (position_sensor() == 1000) {
  //      printSecondRow("junc 1 ");
  //      read_cytron();
  //      print_lcd();
  //    }
  //    while (!(position_sensor() == 1000)) {
  //      printSecondRow("junc 2 ");
  //      read_cytron();
  //      print_lcd();
  //    }
  //
  //  }
  else if (position_sensor() == 10000) {
    printSecondRow("else   ");
    assign_speed(0, 0, 0, 0, 0, 0, 0, 0);


  }


  else if (position_sensor() == 2000) {
    printSecondRow("junc 2 -------");

    print_lcd();
    junc_counter = 1;
    assign_speed(low, 0, 0, 0, randomm_low, 0, 0, 0);
    read_cytron();
    set_speed();
    while (!(position_sensor( ) == 2000)) {
      read_cytron();
    }
    swap_motors();
    swap_cytron();
    printSecondRow("junc 2   swap");
  }
  else {

    assign_speed(0, 0, 0, 0, 0, 0, 0, 0);

  }

  //  Serial.println(position_sensor());
  //  lcd.clear();

}

