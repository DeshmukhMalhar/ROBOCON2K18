
//const int out1 = 1, out2 = 2, out3 = 3, tz1 = 4, tz2 = 5, tz3 = 6;
int current = 0, next = 0;

//shows junc is present or not
bool junction = false;

//Reading values of cytron
uint8_t cytron_read;

//Keep the count of the junc
int junction_counter = 0;

//motor pwms
int low = 17, medium = 22, high = 23, normal = 70;

//offset
int offset_normal[] = {0, 0, -10, -10, 5, 5, 0, 0};
int offset_adj[] = { 0, 0, -6, -6, 4, 4, 0, 0};

//To Mosfet - Each cuople for 1 motor (1,0-Forward)(0,1-Backward)
int motor_pins[] = {2, 3, 4, 5, 6, 7, 10, 11};

//ASSIGNING SPEED TO THE MOTORS
int velocity[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

void print_cytron() {
  Serial.println(PINC, BIN);
}

const bool junction_condition[] = {false, false, false, false, false, false, false, false, false, false, true, false, false, false, false, false, false, false, true, false, true, false, true, false, false, false, true, false, false, false, false, false, false, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, false, false, true, false, true, false, true, false, false, false, true, false, false, false, true, false, false, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, false, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, false, false, true, false, true, false, true, false, false, false, true, false, true, false, true, false, false, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, false, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, false, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false, false, false, true, false, true, false, true, false, true, false, true, false, true, false, true, false};

void set_speed() {
  for (int i = 0; i < 8; i++) {
    analogWrite(motor_pins[i],
                velocity[i] == 0 ? 0 : (velocity[i] == normal ? velocity[i] + offset_normal[i] : velocity[i] +
                                        offset_adj[i]));
  }
}


//Initialization of pins
void initialization() {
  for (int i = 0; i < 8; i++) {
    pinMode(motor_pins[i], OUTPUT);
    digitalWrite(motor_pins[i], LOW);
  }
}

int counter = 0;

bool is_junction(){
  cytron_read = PINC;
  if(junction_condition[cytron_read]==true){
    return true;
  }
  return false;
}

//bool is_junction() {
//  cytron_read = PINC;
//  int temp = cytron_read;
//
//  counter = 0;
//
//  if (cytron_read % 2 == 0) {
//    while (cytron_read % 2 == 0 && cytron_read != 0) {
//      cytron_read >>= 1;
//    }
//    if (cytron_read % 2 == 1) {
//      while (cytron_read % 2 == 1 && cytron_read != 0) {
//        cytron_read >>= 1;
//        counter++;
//      }
//      if (counter >= 5) {
//        return true;
//      }
//      if (cytron_read % 2 == 0) {
//        while (cytron_read % 2 == 0 && cytron_read != 0) {
//          cytron_read >>= 1;
//        }
//        if (cytron_read % 2 == 1) {
//          while (cytron_read % 2 == 1) {
//            cytron_read >>= 1;
//          }
//          return true;
//        } else {
//          return false;
//        }
//      } else {
//        return false;
//      }
//    } else {
//      return false;
//    }
//  } else {
//    return false;
//  }
//}

void shuttle_throw() {
  delay(2000);
}

//Vertical adjsutment of bot
void adj_out() {
  cytron_read = PINC;
  print_cytron();
  Serial.print("in adj out");
  set_speed();


  //LEFT adj

  //    small left
  if ((cytron_read | B00011000) == B01111000) {
    velocity[0] = 0;
    velocity[1] = low;  //motor 1
    velocity[4] = 0;
    velocity[5] = low;  //motor 3
    Serial.println("small left");

  }
  //medium left
  if ((cytron_read | B00100000) == B11100000) {
    velocity[0] = 0;
    velocity[1] = medium;  //motor1
    velocity[4] = 0;
    velocity[5] = medium;  //motor3
    Serial.println("medium left");
  }
  //large left
  else if ((cytron_read | B01110000) == B11110000) {
    velocity[0] = 0;
    velocity[1] = high;  //motor 1
    velocity[4] = 0;
    velocity[5] = high;  //motor 3
    Serial.println("large left");
  }

  //RIGHT adj
  //small right
  else if ((cytron_read | B00011000) == B00011110) {
    velocity[0] = low; //motor 1
    velocity[1] = 0;
    velocity[4] = low;
    velocity[5] = 0;  //motor 3
    Serial.println("small right");
  }

  //medium right
  else if ((cytron_read | B00000100) == B00000111) {
    velocity[0] = medium; //motor 1
    velocity[1] = 0;
    velocity[4] = medium;
    velocity[5] = 0;     //motor 3
    Serial.println("medium right");
  }
  //large right
  else if ((cytron_read | B00000010) == B00000011) {
    velocity[0] = high; //motor 1
    velocity[1] = 0;
    velocity[4] = high;
    velocity[5] = 0;  //motor 3
    Serial.println("large right");
  }

  //junction start
  //  if (cytron_read >= 128 && (((cytron_read | B00101000) == B10111000) || ((cytron_read | B00110000) == B10111000) || ((cytron_read | B01000100) == B11001100) || ((cytron_read | B01000100) == B11011100))) {
  if (is_junction()) {
    //if ((cytron_read | B00101000) == 10111000) {
    //  if((cytron_read==B11110000)||(cytron_read==B01111000)||(cytron_read==B00111100)){
    Serial.print("In junction start");

    if (junction == false) {
      junction_counter++;
    }
    junction = true;
    //    stop1();
    //    set_speed();
    //    while (1) {
    print_cytron();
    // }

  }

}

//Horizontal adjustment of bot
void adj_zone() {
  Serial.println("Zone");
  cytron_read = PINC;
  set_speed();
  //Forward adj

  //small forward
  if ((cytron_read | B00011000) == B01111000) {
    velocity[2] = low; //motor 2
    velocity[3] = 0;
    velocity[6] = low;
    velocity[7] = 0;  //motor 4
  }
  //medium forward
  else if ((cytron_read | B00100000) == B11100000) {
    velocity[2] = medium; //motor 2
    velocity[3] = 0;
    velocity[6] = medium;
    velocity[7] = 0;  //motor 4
  }
  //large forward
  else if ((cytron_read | B01000000) == B11000000) {
    velocity[2] = high; //motor 2
    velocity[3] = 0;
    velocity[6] = high;
    velocity[7] = 0;  //motor 4
  }
  ////////////////////////////////////////////////

  //BACKWARD adj

  //small backward
  else if ((cytron_read | B00011000) == B00011110) {
    velocity[2] = 0; //motor 2
    velocity[3] = low;
    velocity[6] = 0;
    velocity[7] = low;  //motor 4
  }
  //medium backward
  else if ((cytron_read | B00000100) == B00000111) {
    velocity[2] = 0; //motor 2
    velocity[3] = medium;
    velocity[6] = 0;
    velocity[7] = medium;  //motor 4
  }
  //large backward
  else if ((cytron_read | B00000010) == B00000011) {
    velocity[2] = 0; //motor 2
    velocity[3] = high;
    velocity[6] = 0;
    velocity[7] = high;  //motor 4
  }
  //  //junction start
  //  else if ((cytron_read | B00101000) == B10111000) {
  //    junction = true;
  //  }
  //  //junction end
  //  else if ((cytron_read | 00111111) == B00111111) {
  //    junction = false;
  //  }

  if (is_junction()) {
    //if ((cytron_read | B00101000) == 10111000) {
    //  if((cytron_read==B11110000)||(cytron_read==B01111000)||(cytron_read==B00111100)){
    Serial.print("In junction start");

    if (junction == false) {
      junction_counter++;

    }
    junction = true;
    //    stop1();
    //    set_speed();
    //    while (1) {
    //print_cytron();
    // }

  }
  if (!(is_junction())) {
    junction = false;
  }
}


void forward() {

  velocity[2] = normal;
  velocity[3] = 0;
  velocity[6] = normal;
  velocity[7] = 0;
  set_speed();
}

void backward() {
  velocity[2] = 0;
  velocity[3] = normal;
  velocity[6] = 0;
  velocity[7] = normal;
  set_speed();
}

void left() {
  velocity[0] = 0;
  velocity[1] = normal;
  velocity[4] = 0;
  velocity[5] = normal;
  set_speed();
}

void right() {

  velocity[0] = normal;
  velocity[1] = 0;
  velocity[4] = normal;
  velocity[5] = 0;
  set_speed();
}

void stop1() {

  velocity[0] = 0;
  velocity[1] = 0;
  velocity[2] = 0;
  velocity[3] = 0;
  velocity[4] = 0;
  velocity[5] = 0;
  velocity[6] = 0;
  velocity[7] = 0;
  set_speed();
}

void setup() {
  current = 1;
  next = 2;
  initialization();
  delay(1000);
  Serial.begin(9600);
}

void loop() {
  set_speed();
  print_cytron();

  if (current == 1 && next == 2) {
    Serial.print("If current1 and next 2");
    forward();
    print_cytron();
    set_speed();
    Serial.println("In case 1");
    // Serial.println(junction);
    while (junction != true) {
      Serial.println("adj_out");
      adj_out();
      set_speed();
      print_cytron();

    }
    Serial.println("forward while");

    current = 2;
    next = 4;
    junction_counter = 0;
  } else if (current == 2 && next == 4) {
    Serial.print("If current2 and next 4");
    left();

    velocity[2] = 0;
    velocity[3] = 0;
    velocity[6] = 0;
    velocity[7] = 0;
    set_speed();

    while (junction_counter != 2) {
      Serial.println("in case 2 while");
      adj_zone();
    }
    stop1();
    shuttle_throw();
    current = 4;
    next = 2;
  } else if (current == 4 && next == 2) {
    Serial.print("If current4 and next 2");
    right();
    while (junction != true) {
      adj_zone();
    }
    while (junction == true) {
      adj_zone();
    }
    junction_counter = 0;

    while (junction_counter != 2) {
      adj_zone();
    }
    stop1();
    while (1);
    current = 2;
    next = 3;
    junction_counter = 0;
  } else if (current == 2 && next == 3) {
    Serial.print("If current2 and next 3");
    forward();
    while (junction_counter != 1) {
      adj_out();
    }
    //    while (junction != false) {
    //      adj_out();
    //    }
    current = 3;
    next = 5;
    junction_counter = 0;
  } else if (current == 3 && next == 5) {
    Serial.print("If current 3 and next 5");
    left();
    while (junction_counter != 2) {
      adj_zone();
    }
    stop1();
    shuttle_throw();
    current = 5;
    next = 3;
    junction_counter = 0;
  } else if (current == 5 && next == 3) {
    Serial.print("If current5 and next 3");
    right();
    while (junction_counter != 2) {
      adj_zone();
    }
    stop1();
    current = 3;
    next = 6;
    junction_counter = 0;
  } else if (current == 3 && next == 6) {
    Serial.print("If current3 and next 6");
    left();
    while (junction_counter != 5) {
      adj_zone();
    }
    stop1();
    shuttle_throw();
    current = 6;
    next = 6;
    junction_counter = 0;
  } else if (current == 6 && next == 6) {
    stop1();
  } else {
    Serial.print("Should never go");
  }


}
