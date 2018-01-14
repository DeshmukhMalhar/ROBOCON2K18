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
#define m4CW  OCR2A  //

#define m4ACW OCR1A //11
uint16_t swapped_cytron;
uint16_t rotate_byte (uint16_t b ) {
  return (b * 0x0202020202 & 0x010884422010 ) % 1023;
}
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
  Serial.begin(9600);
}

uint8_t cytron_read;
const uint8_t lowest = 30, lower = 35, low = 40, normal = 55, high = 70, random_normal = 53, random_low = 38, random_lower = 34, random_lowest = 30;
int junction_counter = 0;

void loop() {
  //Read from PORT C
  cytron_read = PINC;



  if (junction_counter == 0) {

    while(1){
      m1CW = lower;
      m1ACW = 0;
      m2CW = 0;
      m2ACW = 0;
      m3CW = lower;
      m3ACW = 0;
      m4CW = 0;
      m4ACW = 0;
    }
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
      Serial.println("forward");
    }
    //        Bot is little left
    //        Check if in range b00110000 and b001111000
    else if (cytron_read >= B00110000 && cytron_read <= B00111000) {
      m1CW = lowest;
      m1ACW = 0;
      m2CW = random_normal;
      m2ACW = 0;
      m3CW = lowest;
      m3ACW = 0;
      m4CW = normal;
      m4ACW = 0;
      Serial.println("lit left");
    }
    //        Bot is more left
    //        Check if in range b01100000 and b011111000
    else if (cytron_read >= B0110000 && cytron_read <= B01111000) {
      m1CW = lower;
      m1ACW = 0;
      m2CW = random_normal;
      m2ACW = 0;
      m3CW = lower;
      m3ACW = 0;
      m4CW = normal;
      m4ACW = 0;
      Serial.println("more left");
    }
    //        Bot is most left
    //        Check if in range b11000000 and b111111000
    else if (((cytron_read | B00100000) == B11100000) || ((cytron_read == B10000000))) {
      m1CW = low;
      m1ACW = 0;
      m2CW = random_normal;
      m2ACW = 0;
      m3CW = low;
      m3ACW = 0;
      m4CW = normal;
      m4ACW = 0;
      Serial.println("most left");
    }

    //        Bot is little right
    //        Check if in range b00110000 and b001111000
    else if (((cytron_read | B00010000) == B00011100)) {
      m1CW = 0;
      m1ACW = lowest;
      m2CW = random_normal;
      m2ACW = 0;
      m3CW = 0;
      m3ACW = lowest;
      m4CW = normal;
      m4ACW = 0;
      Serial.println("lit right");
    }
    //        Bot is more right
    //        Check if in range b00000110 and b00011110
    else if ((cytron_read | B00001000) == B00001110) {
      m1CW = 0;
      m1ACW = lower;
      m2CW = random_normal;
      m2ACW = 0;
      m3CW = 0;
      m3ACW = lower;
      m4CW = normal;
      m4ACW = 0;
      Serial.println("more right");
    }
    //        Bot is most right
    //        Check if in range b00000011 and b00011111
    else if (((cytron_read | B00000100) == B00000111) || (cytron_read == B00000001)) {
      m1CW = 0;
      m1ACW = low;
      m2CW = random_normal;
      m2ACW = 0;
      m3CW = 0;
      m3ACW = low;
      m4CW = normal;
      m4ACW = 0;
      Serial.println("most right");
    }
    //Junction thingy
    else if (( (cytron_read | B00101100) == B11111100) || ((cytron_read | B00111100) == B10111100))
    {
      //     while(bitRead(PINC,0)==0);
      junction_counter = 1;
      m1CW = low + 5;
      m1ACW = 0;
      m2CW = low + 5;
      m2ACW = 0;
      m3CW = low + 5;
      m3ACW = 0;
      m4CW = low + 5;
      m4ACW = 0;
      Serial.println("1st junction j=1");

      //      m1ACW = 0;
      //      m1CW = 0;
      //      m2CW = normal;
      //      m2ACW = 0;
      //      m3CW = 0;
      //      m3ACW = 0;
      //      m4CW = normal;
      //      m4ACW = 0;
      //      while (!(( (cytron_read | B00101100) == B11111100) || ((cytron_read | B00111100) == B10111100)) || (!(bitRead(PINC, 3) == 1 || bitRead(PINC, 4) == 1)) || bitRead(PINC, 7) == 0)
      while ((PINC | B01100110) == B01100111);

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
      Serial.println("j=0 else stop");
    }
  }


  //////////////////////////////////////////////////////////////////////////////////////////////
  if (junction_counter == 1) {

    swapped_cytron = rotate_byte(cytron_read);
    if (swapped_cytron == B00011000) {
      m1ACW = 0;
      m1CW = normal;
      m2CW = 0;
      m2ACW = 0;
      m3CW = normal;
      m3ACW = 0;
      m4CW = 0;
      m4ACW = 0;
      Serial.println("j=1 forward");
      
    }
    //        Bot is little left
    //        Check if in range b00110000 and b001111000
    else if (swapped_cytron >= B00110000 && swapped_cytron <= B00111000) {
      m1CW = normal;
      m1ACW = 0;
      m2CW = 0;
      m2ACW = lowest-8;
      m3CW = normal;
      m3ACW = 0;
      m4CW = 0;
      m4ACW = lowest;
      Serial.println("j=1 lit left");

    }

    //        Bot is more left
    //        Check if in range b01100000 and b011111000
    else if (swapped_cytron >= B0110000 && swapped_cytron <= B01111000) {
      m1CW = normal;
      m1ACW = 0;
      m2CW = 0;
      m2ACW = lower-8;
      m3CW = normal;
      m3ACW = 0;
      m4CW = 0;
      m4ACW = lower;
      Serial.println("j=1 moreleft");

    }
    //        Bot is most left
    //        Check if in range b11000000 and b111111000
    else if (((swapped_cytron | B00100000) == B11100000) || ((swapped_cytron == B10000000))) {
      m1CW = normal;
      m1ACW = 0;
      m2CW = 0;
      m2ACW = low-8;
      m3CW = normal;
      m3ACW = 0;
      m4CW = 0;
      m4ACW = low;
      Serial.println("j=1 most left");
    }

    //        Bot is little right
    //        Check if in range b00110000 and b001111000
    else if (((swapped_cytron | B00010000) == B00011100))  {
      m1CW = normal;
      m1ACW = 0;
      m2CW = lowest-8;
      m2ACW = 0;
      m3CW = normal;
      m3ACW = 0;
      m4CW = lowest;
      m4ACW = 0;
      Serial.println("j=1 little right");

    }
    //        Bot is more right
    //        Check if in range b00000110 and b00011110
    else if ((swapped_cytron | B00001000) == B00001110) {
      m1CW = normal;
      m1ACW = 0;
      m2CW = lower-8;
      m2ACW = 0;
      m3CW = normal;
      m3ACW = 0;
      m4CW = lower;
      m4ACW = 0;
      Serial.println("j=1 more right");
    }
    //        Bot is most right
    //        Check if in range b00000011 and b00011111
    else if (((swapped_cytron | B00000100) == B00000111) || (swapped_cytron == B00000001)) {
      m1CW = normal;
      m1ACW = 0;
      m2CW = low-8;
      m2ACW = 0;
      m3CW = normal;
      m3ACW = 0;
      m4CW = low;
      m4ACW = 0;
      Serial.println("j=1 most right");
    }
    else if (( (( (swapped_cytron | B00101100) == B11111100) || ((swapped_cytron | B00111100) == B10111100))))
    {

      //      junction_counter = 2;
      m1ACW = 0;
      m1CW = normal;
      m2CW = 0;
      m2ACW = 0;
      m3CW = normal;
      m3ACW = 0;
      m4CW = 0;
      m4ACW = 0;
      Serial.println("Junction 2 ");
      while (!(( (swapped_cytron | B00101100) == B11111100) || ((swapped_cytron | B00111100) == B10111100))) {
        swapped_cytron = rotate_byte(cytron_read);
      }
    }
    //    else if((swapped_cytron==B10000011)||(swapped_cytron==B11000011)||(swapped_cytron==B11000111)||(swapped_cytron==B11100111)||(swapped_cytron==B10000111)){
    //
    //       m1ACW = 0;
    //      m1CW = normal;
    //      m2CW = 0;
    //      m2ACW = 0;
    //      m3CW = normal;
    //      m3ACW = 0;
    //      m4CW = 0;
    //      m4ACW = 0;
    //      }
    //    else if (( (( (swapped_cytron | B00101100) == B11111100) || ((swapped_cytron | B00111100) == B10111100)) )&&junction_counter==2)
    //    {
    //
    //      junction_counter = 3;
    //      m1ACW = 0;
    //      m1CW = 0;
    //      m2CW = 0;
    //      m2ACW = 0;
    //      m3CW = 0;
    //      m3ACW = 0;
    //      m4CW = 0;
    //      m4ACW = 0;
    //      while (1);
    //
    //
    //    }

    else {
      m1CW = 0;
      m1ACW = 0;
      m2CW = 0;
      m2ACW = 0;
      m3CW = 0;
      m3ACW = 0;
      m4CW = 0;
      m4ACW = 0;
      Serial.println("j=1 else stop");
    }
  }
}
