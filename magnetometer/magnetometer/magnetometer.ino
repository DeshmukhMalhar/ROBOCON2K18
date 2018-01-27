
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

uint8_t motor_pins[] = {10, 11, 3, 2, 4, 5, 7, 6};
uint8_t corr_velocity[] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t cytron_read;
uint8_t offset_normal[] = {0, 0, 0, 0, 0, 0, 0, 0};
float headingDegrees, minimum, maximum, variable;
float heading, declinationAngle ;
uint8_t velocity[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int low = 25, medium = 30, high = 30, normal = 70;
uint8_t relayPin = 22;
void initt() {
  for (int i = 0; i < 8; i++) {
    pinMode(motor_pins[i], OUTPUT);
    digitalWrite(motor_pins[i], LOW);
  }




}

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");

}



void set_speed() {


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
  //  Serial.println(headingDegrees);



  if (headingDegrees < minimum) {
    corr_velocity[2] = (minimum - headingDegrees) * 4;
    //    if (headingDegrees > minimum - 7)
    //    {
    //      corr_velocity[2] = 15;
    //
    //    }
    //    else
    //    {
    //
    //      corr_velocity[2] = 30;
    //      //      Serial.println("threshold 10");
    //    }
    corr_velocity[6] = 0;
    corr_velocity[7] = 0;
    //    Serial.println("direction");
  }
  else if (headingDegrees > maximum) {
    corr_velocity[6] = (headingDegrees - maximum) * 4;
    //    if (headingDegrees < maximum + 7)
    //    {
    //
    //      corr_velocity[6] = 15;
    //      //      Serial.println("threshold 5");
    //    }
    //    else
    //    {
    //      corr_velocity[6] = 30;
    //      //      Serial.println("threshold 10");
    //    }
    corr_velocity[2] = 0;
    corr_velocity[3] = 0;
    //    Serial.println("anti direction");
  }
  else {
    corr_velocity[2] = 0;
    corr_velocity[3] = 0;
    corr_velocity[6] = 0;
    corr_velocity[7] = 0;
  }
  for (int i = 0; i < 8; i++) {
    //    Serial.println("in for of set speed");
    analogWrite(motor_pins[i],
                velocity[i] == 0 ? 0 : (velocity[i] == normal ? velocity[i] + corr_velocity[i] : velocity[i]));

    //                Serial.println(velocity[i] + corr_velocity[i]);

  }
  //  Serial.println("pwm2");
  //  Serial.println(pwm2);
  //   Serial.println("pwm4");
  //  Serial.println(pwm4);8
  //for(int i=0;i<8;i++)
  //{
  //      velocity[i] == normal ? Serial.println(velocity[i] + corr_velocity[i]) : Serial.println(velocity[i] );
  //}
}


void adj_out() {

  cytron_read = PINC;
  //LEFT adj

  //    small left
  if ((cytron_read | B00011000) == B01111000) {
    velocity[0] = 0;
    velocity[1] = low;  //motor 1
    velocity[4] = 0;
    velocity[5] = low;  //motor 3
    //    //Serial.println("small left");

  }
  //medium left
  else if ((cytron_read | B00100000) == B11100000) {
    velocity[0] = 0;
    velocity[1] = medium;  //motor1
    velocity[4] = 0;
    velocity[5] = medium;  //motor3
    //    //Serial.println("medium left");
  }
  //large left
  else if ((cytron_read | B01110000) == B11110000) {
    velocity[0] = 0;
    velocity[1] = high;  //motor 1
    velocity[4] = 0;
    velocity[5] = high;  //motor 3
    //    //Serial.println("large left");
  }

  //RIGHT adj
  //small right
  else if ((cytron_read | B00011000) == B00011110) {
    velocity[0] = low; //motor 1
    velocity[1] = 0;
    velocity[4] = low;
    velocity[5] = 0;  //motor 3
    //    //Serial.println("small right");
  }

  //medium right
  else if ((cytron_read | B00000100) == B00000111) {
    velocity[0] = medium; //motor 1
    velocity[1] = 0;
    velocity[4] = medium;
    velocity[5] = 0;     //motor 3
    //    //Serial.println("medium right");
  }
  //large right
  else if ((cytron_read | B00000010) == B00000011) {
    velocity[0] = high; //motor 1
    velocity[1] = 0;
    velocity[4] = high;
    velocity[5] = 0;  //motor 3
    //    //Serial.println("large right");
  }
  set_speed();
}

void forward() {

  velocity[2] = normal;
  velocity[3] = 0;
  velocity[6] = normal;
  velocity[7] = 0;
  set_speed();

}
void setup() {
  // put your setup code here, to run once:
  initt();
  pinMode(22, OUTPUT);
  Serial.begin(9600);
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");

  /* Initialise the sensor */
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();

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


  int currentmillis = millis();
  while (millis() < currentmillis + 5000)
  {

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

    Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
    if (headingDegrees < minimum)
      minimum = headingDegrees;
    else if (headingDegrees > maximum)
      maximum = headingDegrees;
  }
  Serial.println("min=");
  Serial.println(minimum);
  delay(500);
  Serial.println("max=");
  Serial.println(maximum);
  delay(500);



  //  digitalWrite(22, HIGH);
  //
  //
  //  digitalWrite(22, LOW);
}
void loop()
{
  forward();
  adj_out();
  //  for(int i=0;i<8;i++){
  //    Serial.println(corr_velocity[i] );
  //  }

}




