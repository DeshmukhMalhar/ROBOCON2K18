#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
//#define debug

bool t_up = false;
bool t_left = false;
bool t_down = false;
bool t_right = false;
bool t_clk = false;
bool t_aclk = false;
bool s_up = true;
bool s_down = true;
bool s_left = true;
bool s_right = true;
bool s_clk = true;
bool s_aclk = true;

RF24 radio(2, 3); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

void setup() {
    Serial.begin(9600);
    radio.begin();
    radio.openWritingPipe(addresses[1]); // 00001
    radio.openReadingPipe(1, addresses[0]); // 00002

    radio.setPALevel(RF24_PA_MAX);
    radio.stopListening();
    pinMode(7, INPUT);
    pinMode(6, INPUT);
    pinMode(5, INPUT);
    pinMode(4, INPUT);
    pinMode(8, INPUT);
    pinMode(9, INPUT);
}

const char text2[32] = "xyz";

void loop() {

    //////////////////////////////(7)
    if (digitalRead(7) == HIGH) {
        if (t_right == false) {
            Serial.println("sending 7!!!");
            int text = '7';
            radio.stopListening();
            radio.write(&text, sizeof(text));
            radio.startListening();
            t_right = true;
        }
        s_right = true;
    } else {
        if (s_right == true) {
            s_right = false;
            Serial.println("stop");
            radio.stopListening();
            int text = '0';
            radio.write(&text, sizeof(text));
        }
        t_right = false;
    }

    //////////////////////////////(4)
    if (digitalRead(4) == HIGH) {
        if (t_up == false) {
            Serial.println("sending 4!!!");
            int text = '4';
            radio.stopListening();
            radio.write(&text, sizeof(text));
            radio.startListening();
            t_up = true;
        }
        s_up = true;
    } else {
        if (s_up == true) {
            s_up = false;
            Serial.println("stop");
            radio.stopListening();
            int text = '0';
            radio.write(&text, sizeof(text));
        }
        t_up = false;
    }
    //////////////////////////////(5)
    if (digitalRead(5) == HIGH) {
        if (t_down == false) {
            Serial.println("sending 5!!!");
            int text = '5';
            radio.stopListening();
            radio.write(&text, sizeof(text));
            radio.startListening();
            t_down = true;
        }
        s_down = true;
    } else {
        if (s_down == true) {
            s_down = false;
            Serial.println("stop");
            radio.stopListening();
            int text = '0';
            radio.write(&text, sizeof(text));
        }
        t_down = false;
    }

//////////////////////////////(6)
    if (digitalRead(6) == HIGH) {
        if (t_left == false) {
            Serial.println("sending 6!!!");
            int text = '6';
            radio.stopListening();
            radio.write(&text, sizeof(text));
            radio.startListening();
            t_left = true;
        }
        s_left = true;
    } else {
        if (s_left == true) {
            s_left = false;
            Serial.println("stop");
            radio.stopListening();
            int text = '0';
            radio.write(&text, sizeof(text));
        }
        t_left = false;
    }

//////////////////////////////////(for clk)
    if (digitalRead(9) == HIGH) {
        if (t_clk == false) {
            Serial.println("sending clk!!!");
            int text = '9';
            radio.stopListening();
            radio.write(&text, sizeof(text));
            radio.startListening();
            t_clk = true;
        }
        s_clk = true;
    } else {
        if (s_clk == true) {
            s_clk = false;
            Serial.println("stop");
            radio.stopListening();
            int text = '0';
            radio.write(&text, sizeof(text));
        }
        t_clk = false;
    }

    /////////////////////////////////////(for aclk)
    if (digitalRead(8) == HIGH) {
        if (t_aclk == false) {
            Serial.println("sending aclk!!!");
            int text = '8';
            radio.stopListening();
            radio.write(&text, sizeof(text));
            radio.startListening();
            t_aclk = true;
        }
        s_aclk = true;
    } else {
        if (s_aclk == true) {
            s_aclk = false;
            Serial.println("stop");
            radio.stopListening();
            int text = '0';
            radio.write(&text, sizeof(text));
        }
        t_aclk = false;
    }

    //////////////////////////////////
    if (radio.available()) {
        Serial.println("inside listening to receiver..");
        radio.read(&text2, sizeof(text2));
        Serial.println(text2);
        delay(5);
        radio.stopListening();
    }

}
