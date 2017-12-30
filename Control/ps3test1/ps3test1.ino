#include <PS3USB.h>
//                     m1,m1,m2,m2,m3,m3,m4,m4
const int motorPins[] = {2, 3, 4, 5, 6, 8, 1,7};

USB Usb;

PS3USB PS3(&Usb);

void setup() {
  //Serial.begin(115200);
  for (int i = 0; i < 8; i++) {
    pinMode(motorPins[i], OUTPUT);
  }
//  while (!//Serial);
  if (Usb.Init() == -1) {
    //Serial.print(F("\r\nOSC did not start"));
    while (1);
  }
  //Serial.print(F("\r\nPS3 USB Library Started"));
}

void go(int m1clk, int m1aclk, int m2clk, int m2aclk, int m3clk, int m3aclk, int m4clk, int m4aclk) {
  int temp[] = {m1clk, m1aclk, m2clk, m2aclk, m3clk, m3aclk, m4clk, m4aclk};

  for (int i = 0; i < 8; i++) {
    digitalWrite(motorPins[i], temp[i]);
  }
}

void loop() {
  Usb.Task();
  if (PS3.PS3Connected) {
    if (PS3.getButtonPress(UP)) {
      //Serial.println("UP");
      go(1, 0, 0, 0, 1, 0, 0, 0);
    }
    else if (PS3.getButtonPress(DOWN)) {
      //Serial.println("DOWN");
      go(0, 1, 0, 0, 0, 1, 0, 0);

    }
    else if (PS3.getButtonPress(LEFT)) {
      //Serial.println("LEFT");
      go(0, 0, 1, 0, 0, 0, 1, 0);
    }
    else if (PS3.getButtonPress(RIGHT)) {
      //Serial.println("RIGHT");
      go(0, 0, 0, 1, 0, 0, 0, 1);
    }
    else {
      //Serial.println("Stopper");
      go(0, 0, 0, 0, 0, 0, 0, 0);
    }
  }
  else {
    //Serial.println("Disconnected");
    go(0, 0, 0, 0, 0, 0, 0, 0);
  }

}
