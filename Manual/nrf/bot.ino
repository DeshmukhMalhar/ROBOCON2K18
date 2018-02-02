#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(22, 23); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};
//const char checko[32] = "there!";
//char checki[32] = "";



void setup() {
    Serial.begin(9600);
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);

    radio.begin();
    radio.setAutoAck(1);
    radio.setPALevel(RF24_PA_MAX);
    //  radio.setCRCLength(RF24_CRC_16);
    radio.startListening();
    radio.printDetails();
    radio.openWritingPipe(addresses[0]); // 00002
    radio.openReadingPipe(1, addresses[1]); // 00001

}

char text[32] = "";


void loop() {

    if (radio.available()) {
        radio.read(&text, sizeof(text));
        Serial.println(text);
        /////////////////////////////////////////////(for 7)
        if (strcmp("7", text) == 0) {
            radio.stopListening();
            char text2[32] = "got 7";
            radio.write(&text2, sizeof(text2));
            Serial.println("sending got 7!!");
            radio.startListening();
        }

        /////////////////////////////////////////////(for 6)
        if (strcmp("6", text) == 0) {

            radio.stopListening();
            char text2[32] = "got 6";
            radio.write(&text2, sizeof(text2));
            Serial.println("sending got 6!!");
            delay(100);
            radio.startListening();
        }
        /////////////////////////////////////////////(for 4)
        if (strcmp("4", text) == 0) {
            radio.stopListening();
            char text2[32] = "got 4";
            radio.write(&text2, sizeof(text2));
            Serial.println("sending got 4!!");
            delay(100);
            digitalWrite(2, HIGH);
            digitalWrite(3, LOW);
            radio.startListening();
        }
        /////////////////////////////////////////////(for 5)
        if (strcmp("5", text) == 0) {
            radio.stopListening();
            char text2[32] = "got 5";
            radio.write(&text2, sizeof(text2));
            Serial.println("sending got 5!!");
            delay(100);
            digitalWrite(2, LOW);
            digitalWrite(3, HIGH);
            radio.startListening();
        }
        /////////////////////////////////////////////(for 8)
        if (strcmp("8", text) == 0) {

            radio.stopListening();
            char text2[32] = "got 8";
            radio.write(&text2, sizeof(text2));
            Serial.println("sending got 8!!");
            delay(100);
            radio.startListening();
        }
        /////////////////////////////////////////////(for 9)
        if (strcmp("9", text) == 0) {
            radio.stopListening();
            char text2[32] = "got 9";
            radio.write(&text2, sizeof(text2));
            Serial.println("sending got 9!!");
            delay(100);
            radio.startListening();
        }
        /////////////////////////////////
        if (strcmp("0", text) == 0) {
            digitalWrite(2, HIGH);
            digitalWrite(3, HIGH);
            radio.stopListening();
            char text2[32] = "got 0";
            radio.write(&text2, sizeof(text2));
            Serial.println("sending got 0!!");
            delay(100);
            radio.startListening();
        } else {
            delay(0);

        }
    }
}