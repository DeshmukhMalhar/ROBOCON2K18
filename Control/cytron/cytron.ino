//Readings from cytron stored in this
int readings[] = { -10, -10, -10, -10, -10, -10, -10, -10};

//Speed of each wheel
int speed_wheel[] = {0, 0, 0, 0, 0, 0, 0, 0};

//To Mosfet - Each cuople for 1 motor (1,0-Forward)(0,1-Backward)
int motor_pins[] = {2, 3, 4, 5, 6, 7, 8, 9};

//Sensor pins from Cytron
int cytron_pins[] = {30, 31, 32, 33, 34, 35, 36, 37};

//Initialization of pins
void init() {
  for (int i = 0; i < 8; i++) {
    pinMode(motor_pins[i], OUTPUT);
    digitalWrite(motor_pins[i], LOW);
    pinMode(cytron_pins[i], INPUT);
  }
}
int white = HIGH, black = LOW;
void set_speed() {
  for (int i = 0; i < 8; i++) {
    analogWrite(motor_pins[i], speed_wheel[i]);
  }

}
void assign_speed(int motor1CW, int motor1CCW, int motor2CW, int motor2CCW, int motor3CW, int motor3CCW , int motor4CW , int motor4CCW) {
  int temp[] = {motor1CW, motor1CCW, motor2CW, motor2CCW, motor3CW, motor3CCW, motor4CW, motor4CCW};
  for (int i = 0; i < 8; i++) {
    speed_wheel[i] = temp[i];
  }


}
void setup() {
  init();

}


//Read from Cytron
void read_cytron() {
  for (int i = 0; i < 8; i++) {
    readings[i] = digitalRead(cytron_pins[i]);
  }
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
  if (readings[0] == white  && readings[2] == black && readings[3] == black && readings[4] == black && readings[5] == black && readings[6] == black && readings[7] == black) {
    return -3;//lefttt
  }
  if (readings[0] == black && readings[1] == white && readings[2] == white && readings[3] == black && readings[4] == black && readings[5] == black && readings[6] == black && readings[7] == black) {
    return -2;//leftt
  }
  if (readings[0] == black && readings[1] == black && readings[2] == white && readings[3] == white && readings[4] == black && readings[5] == black && readings[6] == black && readings[7] == black) {
    return -1;//left
  }
  if (readings[0] == black && readings[1] == black && readings[2] == black && readings[3] == white && readings[4] == white && readings[5] == black && readings[6] == black && readings[7] == black) {
    return 0;//centre
  }
  if (readings[0] == black && readings[1] == black && readings[2] == black && readings[3] == black && readings[4] == white && readings[5] == white && readings[6] == black && readings[7] == black) {
    return 1;//right
  }
  if (readings[0] == black && readings[1] == black && readings[2] == black && readings[3] == black && readings[4] == black && readings[5] == white && readings[6] == white && readings[7] == black) {
    return 2;//rightt
  }
  if (readings[0] == black && readings[1] == black && readings[2] == black && readings[3] == black && readings[4] == black && readings[5] == black  && readings[7] == white) {
    return 3;//righttt
  }
  if (readings[0] == white && readings[1] == white && readings[2] == white && readings[3] == white && readings[4] == white && readings[5] == black && readings[6] == black && readings[7] == black) {
    return -100;//left junction
  }
  if (readings[0] == black && readings[1] == black && readings[2] == black && readings[3] == white && readings[4] == white && readings[5] == white && readings[6] == white && readings[7] == white) {
    return 100;//right junction
  }
  if (readings[0] == white && readings[1] == white && readings[2] == white && readings[3] == black && readings[4] == white && readings[5] == white && readings[6] == white && readings[7] == white) {
    return 1000;//all white
    if (readings[0] == black && readings[1] == black && readings[2] == black && readings[3] == black && readings[4] == black && readings[5] == black && readings[6] == black && readings[7] == black) {
      return 10000;//all black
    }

  }
}

void loop() {
  read_cytron();
  set_speed();
  if (position_sensor() == 0) {
    assign_speed(0, 0, 125, 0, 0, 0, 125, 0);
    Serial.println("0");

  }
  if (position_sensor() == -1) {
    assign_speed(55, 0, 0, 0, 55, 0, 0, 0);
Serial.println("-1");

  }
  if (position_sensor() == -2) {
    assign_speed(70, 0, 0, 0, 70, 0, 0, 0);
Serial.println("-2");
  }

  if (position_sensor() == -3) {
    assign_speed(90, 0, 0, 0, 90, 0, 0, 0);
Serial.println("-3");
  }

  if (position_sensor == 1) {
    assign_speed(0, 55, 0, 0, 0, 55, 0, 0);
Serial.println("1");

  }
  if (position_sensor == 2) {
    assign_speed(0, 70, 0, 0, 0, 70, 0, 0);
Serial.println("2");
  }
  if (position_sensor == 3) {
    assign_speed(0, 90, 0, 0, 0, 90, 0, 0);
Serial.println("3");
  }
  if (position_sensor == 10000) {
    assign_speed(0, 0, 0, 0, 0, 0, 0, 0);
Serial.println("10000");
  }



}
