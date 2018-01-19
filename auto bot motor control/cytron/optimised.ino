/*2  OCR3B  PE4
  5  OCR3A  PE3
  6  OCR4A  PH3
  7  OCR4B  PH4
  8  OCR4C  PH5
  9  OCR2B  PH6
  10 OCR2A  PB4
  11 OCR2B  PB5
*/

#define setB(m, n) (m|=(1<<n))


#define m1ACW OCR3A //5
#define m1CW  OCR3B  //2
#define m2CW  OCR4A  //6
#define m2ACW OCR4B //7
#define m3CW  OCR4C  //8
#define m3ACW OCR2B //9
#define m4CW  OCR2A  //10
#define m4ACW OCR1A //11

void initialization() {
  TCCR2A = 0b10100001;
  TCCR2B = 0b00000100;

  TCCR1A = 0b10101001;
  TCCR1B = 0b00000011;
  TCCR1C = 0b00000000;

  TCCR3A = 0b10101001;
  TCCR3B = 0b00000011;
  TCCR3C = 0b00000000;

  TCCR4A = 0b10101001;
  TCCR4B = 0b00000011;
  TCCR4C = 0b00000000;

  DDRC = 0x00;

  DDRE |= (1 << PE4) | (1 << PE3);//DDRE=0b00011000
  DDRH = 0b01111000;
  DDRB |= (1 << PB4) | (1 << PB5);
}


void setup() {
  initialization();
}

uint8_t cytron_read;
const uint8_t lowest = 30, lower = 35, low = 40, normal = 40, high = 70, random_speed = 37;

void loop() {
  //Read from PORT C
  cytron_read = PINC;

  //    Forward
  if (cytron_read == B00011000) {
    m1ACW = 0;
    m1CW = 0;
    m2CW = normal;
    m2ACW = 0;
    m3CW = 0;
    m3ACW = 0;
    m4CW = normal;
    m4ACW = 0;
  }
  //        Bot is little left
  //        Check if in range b00110000 and b001111000
  else if (cytron_read >= B00110000 && cytron_read <= B00111000) {
    m1CW = lowest;
    m1ACW = 0;
    m2CW = normal;
    m2ACW = 0;
    m3CW = lowest;
    m3ACW = 0;
    m4CW = normal;
    m4ACW = 0;
  }
  //        Bot is more left
  //        Check if in range b01100000 and b011111000
  else if (cytron_read >= B0110000 && cytron_read <= B01111000) {
    m1CW = lower;
    m1ACW = 0;
    m2CW = random_speed;
    m2ACW = 0;
    m3CW = lower;
    m3ACW = 0;
    m4CW = normal;
    m4ACW = 0;
  }
  //        Bot is most left
  //        Check if in range b11000000 and b111111000
  else if (cytron_read >= B11000000 && cytron_read <= B11111000) {
    m1CW = low;
    m1ACW = 0;
    m2CW = random_speed;
    m2ACW = 0;
    m3CW = low;
    m3ACW = 0;
    m4CW = normal;
    m4ACW = 0;
  }

  //        Bot is little right
  //        Check if in range b00110000 and b001111000
  else if (cytron_read >= B00001100 && cytron_read <= B00011100) {
    m1CW = 0;
    m1ACW = lowest;
    m2CW = random_speed;
    m2ACW = 0;
    m3CW = 0;
    m3ACW = lowest;
    m4CW = normal;
    m4ACW = 0;
  }
  //        Bot is more right
  //        Check if in range b00000110 and b00011110
  else if (cytron_read >= B00000110 && cytron_read <= B00011110) {
    m1CW = 0;
    m1ACW = lower;
    m2CW = random_speed;
    m2ACW = 0;
    m3CW = 0;
    m3ACW = lower;
    m4CW = normal;
    m4ACW = 0;
  }
  //        Bot is most right
  //        Check if in range b00000011 and b00011111
  else if (cytron_read >= B00000011 && cytron_read <= B00011111) {
    m1CW = 0;
    m1ACW = low;
    m2CW = random_speed;
    m2ACW = 0;
    m3CW = 0;
    m3ACW = low;
    m4CW = normal;
    m4ACW = 0;
  }
  //    Junction
  else if ((cytron_read | B10000001) == B11111111) {
    //        Left 360 until junction is gone
//    m1CW = 0;
//    m1ACW = 0;
//    m2CW = 0;
//    m2ACW = 0;
//    m3CW = 0;
//    m3ACW = 0;
//    m4CW = 0;
//    m4ACW = 0;

//    while (1);


    while ((cytron_read | B10000001) == B11111111) {

      m1CW = normal;
      m1ACW = 0;
      m2CW = random_speed;
      m2ACW = 0;
      m3CW = 0;
      m3ACW = normal;
      m4CW = 0;
      m4ACW = normal;
      cytron_read = PINC;
    }

    while (!((cytron_read | B10000001) == B11111111)) {
      m1CW = normal;
      m1ACW = 0;
      m2CW = random_speed;
      m2ACW = 0;
      m3CW = 0;
      m3ACW = normal;
      m4CW = 0;
      m4ACW = normal;
      cytron_read = PINC;
    }

  }
  else {

    m1CW = 0;
    m1ACW = 0;
    m2CW = 0;
    m2ACW = 0;
    m3CW = 0;
    m3ACW = 0;
    m4CW = 0;
    m4ACW = 0;
  }


}

