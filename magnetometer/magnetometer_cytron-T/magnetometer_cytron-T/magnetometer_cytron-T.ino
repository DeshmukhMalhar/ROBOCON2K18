#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "Arduino.h"

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

const uint8_t Kp = 3.5;
uint8_t speedo = 0;
uint8_t motor_pins[] = {10, 11, 3, 2, 4, 5, 7, 6};
uint8_t corr_velocity[] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t sensor1, sensor2;
uint8_t offset_normal[] = {0, 0, 0, 0, 0, 0, 0, 0};
float headingDegrees, minimum, maximum;
float heading, declinationAngle;
uint8_t velocity[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t low = 27, medium = 30, high = 32, normal = 70;
uint8_t relayPin = 22;
uint8_t previous_state, counter_decider;
uint8_t junction_counter = 0;
uint8_t higher = 70;

uint8_t junction_counter = 0;

bool junction = false;

uint8_t current = 1, next = 2;
int mz1, mz2, mz3, mz4;
int mo1, mo2, mo3, mo4;

void initt() {
    for (int i = 0; i < 8; i++) {
        pinMode(motor_pins[i], OUTPUT);
        digitalWrite(motor_pins[i], LOW);
    }
}

void shuttle_throw() {

    delay(2000);
}

void printSensor() {
    Serial.print("Sensor 1 =");
    Serial.println(PINC, BIN);
    Serial.print("Sensor 2 =");
    Serial.println(PINL, BIN);
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
    }
//    No need for else
    return false;

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

    if (headingDegrees < minimum) {
        speedo = (minimum - headingDegrees) * Kp;
        if (speedo > 50) {
            speedo = 50;
        }
        corr_velocity[mo1] = speedo;

        corr_velocity[mo3] = 0;
        corr_velocity[mo4] = 0;

    } else if (headingDegrees > maximum) {
        speedo = (headingDegrees - maximum) * Kp;
        if (speedo > 50) {
            speedo = 50;
        }
        corr_velocity[mo3] = speedo;

        corr_velocity[mo1] = 0;
        corr_velocity[mo2] = 0;


    } else {
        corr_velocity[2] = 0;
        corr_velocity[3] = 0;
        corr_velocity[6] = 0;
        corr_velocity[7] = 0;
    }
    for (int i = 0; i < 8; i++) {

        analogWrite(motor_pins[i],
                    velocity[i] == 0 ? 0 : (velocity[i] == normal ? velocity[i] + corr_velocity[i] : velocity[i]));
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

    if (headingDegrees < minimum) {
        ////Serial.println("LEft");
        speedo = (minimum - headingDegrees) * Kp;
        if (speedo > 50) {
            speedo = 50;
        }

        corr_velocity[mz1] = speedo;

        corr_velocity[mz3] = 0;
        corr_velocity[mz4] = 0;

    } else if (headingDegrees > maximum) {
        ////Serial.println("Right");
        speedo = (headingDegrees - maximum) * Kp;
        if (speedo > 50) {
            speedo = 50;
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
    for (int i = 0; i < 8; i++) {
        analogWrite(motor_pins[i],
                    velocity[i] == 0 ? 0 : (velocity[i] == normal ? velocity[i] + corr_velocity[i] : velocity[i]));
    }

}

void adj_out() {

    sensor1 = PINC;
    //LEFT adj

    //    small left
    if ((sensor1 | B00011000) == B01111000) {
        velocity[0] = 0;
        velocity[1] = low;  //motor 1
        velocity[4] = 0;
        velocity[5] = low;  //motor 3
        //    ////Serial.println("small left");

    }
        //medium left
    else if ((sensor1 | B00100000) == B11100000) {
        velocity[0] = 0;
        velocity[1] = medium;  //motor1
        velocity[4] = 0;
        velocity[5] = medium;  //motor3
        //    ////Serial.println("medium left");
    }
        //large left
    else if ((sensor1 | B01110000) == B11110000) {
        velocity[0] = 0;
        velocity[1] = high;  //motor 1
        velocity[4] = 0;
        velocity[5] = high;  //motor 3
    }

        //RIGHT adj
        //small right
    else if ((sensor1 | B00011000) == B00011110) {
        velocity[0] = low; //motor 1
        velocity[1] = 0;
        velocity[4] = low;
        velocity[5] = 0;  //motor 3
    }

        //medium right
    else if ((sensor1 | B00000100) == B00000111) {
        velocity[0] = medium; //motor 1
        velocity[1] = 0;
        velocity[4] = medium;
        velocity[5] = 0;     //motor 3
        //    ////Serial.println("medium right");
    }
        //large right
    else if ((sensor1 | B00000010) == B00000011) {
        velocity[0] = high; //motor 1
        velocity[1] = 0;
        velocity[4] = high;
        velocity[5] = 0;  //motor 3
        //    ////Serial.println("large right");
    } else if ((sensor1 == B00011000)) {
        velocity[0] = 0; //motor 1
        velocity[1] = 0;
        velocity[4] = 0;
        velocity[5] = 0;
    }

    if (is_junction_out()) {
        if (junction == false) {
            junction_counter++;
        }
        junction = true;
    } else junction = false;
    set_speed_out();
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
    mz1 = 1;
    mz3 = 5;
    mz2 = 0;
    mz4 = 4;
    velocity[0] = 0;
    velocity[1] = normal;
    velocity[4] = 0;
    velocity[5] = normal;
    set_speed_zone();
}

void right() {
    mz1 = 4;
    mz3 = 0;
    mz2 = 5;
    mz4 = 1;

    velocity[0] = normal;
    velocity[1] = 0;
    velocity[4] = normal;
    velocity[5] = 0;
    set_speed_zone();
}

void adj_zone() {
    ////Serial.println("Zone");
    sensor2 = PINL;

    //Forward adj

    //small forward
    if ((sensor2 | B00011000) == B01111000) {
        velocity[3] = low; //motor 2
        velocity[2] = 0;
        velocity[7] = low;
        velocity[6] = 0;  //motor 4
    }
        //medium forward
    else if ((sensor2 | B00100000) == B11100000) {
        velocity[3] = medium; //motor 2
        velocity[2] = 0;
        velocity[7] = medium;
        velocity[6] = 0;  //motor 4
    }
        //large forward
    else if ((sensor2 | B01000000) == B11000000) {
        velocity[3] = high; //motor 2
        velocity[2] = 0;
        velocity[7] = high;
        velocity[6] = 0;  //motor 4
    }
        //BACKWARD adj
        //small backward
    else if ((sensor2 | B00011000) == B00011110) {
        velocity[3] = 0; //motor 2
        velocity[2] = low;
        velocity[7] = 0;
        velocity[6] = low;  //motor 4
    }
        //medium backward
    else if ((sensor2 | B00000100) == B00000111) {
        velocity[3] = 0; //motor 2
        velocity[2] = medium;
        velocity[7] = 0;
        velocity[6] = medium;  //motor 4
    }
        //large backward
    else if ((sensor2 | B00000010) == B00000011) {
        velocity[3] = 0; //motor 2
        velocity[2] = high;
        velocity[7] = 0;
        velocity[6] = high;  //motor 4
    } else if (sensor2 == B00011000) {
        velocity[3] = 0; //motor 2
        velocity[2] = 0;
        velocity[7] = 0;
        velocity[6] = 0;
    }
    if (is_junction_zone()) {
        if (junction == false) {
            junction_counter++;
        }
        junction = true;
    } else junction = false;

    set_speed_zone();
}

void set_stopper() {
    for (int i = 0; i < 8; i++) {
        digitalWrite(motor_pins[i], 0);
    }
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
    set_stopper();
}

void setup() {
    initt();

    /* Initialise the sensor */
    if (!mag.begin()) {
        /* There was a problem detecting the HMC5883 ... check your connections */
        //Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
        while (1);
    }

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
    minimum = headingDegrees;

    maximum = headingDegrees;


    long currentmillis = millis();
    while (millis() < currentmillis + 5000) {

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

        if (headingDegrees < minimum)
            minimum = headingDegrees;
        else if (headingDegrees > maximum)
            maximum = headingDegrees;
    }
    delay(500);
}

void loop() {

    if (current == 1 && next == 2) {
        forward();
        while (junction != true) {
            adj_out();
        }
        while (junction == true) {
            if ((sensor1 | B00011000) == B01111000 || (sensor1 | B00100000) == B11100000 ||
                (sensor1 | B01110000) == B11110000 || sensor1 == B00011000) {
                previous_state = -1;
            } else {
                previous_state = 1;
            }

            adj_out();
        }
        current = 2;
        next = 4;
        junction_counter = 0;

    } else if (current == 2 && next == 4) {
        velocity[2] = 0;
        velocity[3] = 0;
        velocity[6] = 0;
        velocity[7] = 0;
        if (previous_state == -1) {
            counter_decider = 2;
        } else if (previous_state == 1) {
            counter_decider = 3;
        }
        left();
        adj_zone();
        while (junction_counter != counter_decider) {
            adj_zone();
        }
        stop1();
        shuttle_throw();

        current = 4;
        next = 2;
        junction_counter = 0;

    } else if (current == 4 && next == 2) {
        right();
        adj_zone();
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
        current = 2;
        next = 3;
        junction_counter = 0;

    } else if (current == 2 && next == 3) {
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
        junction_counter = 0;
    } else if (current == 3 && next == 5) {
        velocity[2] = 0;
        velocity[3] = 0;
        velocity[6] = 0;
        velocity[7] = 0;
        left();

        while (junction == true) {
            adj_zone();
        }
        junction_counter = 0;
        while (junction_counter != 2) {
            adj_zone();

        }
        stop1();
        shuttle_throw();
        current = 5;
        next = 3;
        junction_counter = 0;
    } else if (current == 5 && next == 3) {
        right();
        adj_zone();
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
        shuttle_throw();
        current = 3;
        next = 6;
    } else if (current == 3 && next == 6) {
        velocity[2] = 0;
        velocity[3] = 0;
        velocity[6] = 0;
        velocity[7] = 0;
        left();
        adj_zone();
        while (junction == false) {
            adj_zone();
        }
        while (junction == true) {
            adj_zone();
        }
        junction_counter = 0;
        while (junction_counter != 5) {
            adj_zone();

        }
        stop1();
        while (1);
    }


}

void test(){

}



