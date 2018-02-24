#define trigPinO 52
#define echoPinO A13
#define trigPinZ 50
#define echoPinZ A14
#define arm1 23
#define arm2 25
#define arm3 27
#define ultra_out 17
#define ultra_zone 18
void setup() {
  // put your setup code here, to run once:
  pinMode(arm1, INPUT);
  pinMode(arm2, INPUT);
  pinMode(arm3, INPUT);
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(ultra_out, OUTPUT);
  pinMode(ultra_zone, OUTPUT);
  pinMode(trigPinO, OUTPUT);
  pinMode(echoPinO, INPUT);
  pinMode(trigPinZ, OUTPUT);
  pinMode(echoPinZ, INPUT);
  Serial.begin(115200);

}
/*
   23




*/
void loop() {
  //   put your main code here, to run repeatedly:
  //      Serial.print("digitalRead(arm1)");
  //      Serial.println(digitalRead(arm1));
  //      Serial.print("digitalRead(arm2)");
  //      Serial.println(digitalRead(arm2));
  //      Serial.print("digitalRead(arm3)");
  //      Serial.println(digitalRead(arm3));


  if (!digitalRead(arm1)) {
    digitalWrite(14, 1);

  }
  else digitalWrite(14, 0);

  if (!digitalRead(arm2)) {
    digitalWrite(15, 1);

  }
  else digitalWrite(15, 0);

  if (!digitalRead(arm3)) {
    digitalWrite(16, 1);

  }
 
  else digitalWrite(16, 0);
  
  long durationO, distanceO;
  digitalWrite(trigPinO, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinO, HIGH);
  //  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinO, LOW);
  durationO = pulseIn(echoPinO, HIGH);
  distanceO = (durationO / 2) / 29.1;
//    Serial.println(distanceO);
  if (distanceO >=130 ){
    digitalWrite(17,HIGH);
//     Serial.println("h");
  }
  else {
    digitalWrite(17,LOW);
//     Serial.println("l");
  }

    long durationZ, distanceZ;
  digitalWrite(trigPinZ, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinZ, HIGH);
  //  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinZ, LOW);
  durationZ = pulseIn(echoPinZ, HIGH);
  distanceZ = (durationZ / 2) / 29.1;
  if(distanceZ>=130){
    digitalWrite(18,HIGH);
//         Serial.println("h");
  }
  else {
    digitalWrite(18,LOW);
//         Serial.println("l");
  }
//    Serial.println(distanceZ);


}


