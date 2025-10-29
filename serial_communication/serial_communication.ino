#include <Servo.h>

int pins[4] = {3,5,6,9};
String val;

//Declare servos
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

Servo servos[4] = {servo1, servo2, servo3, servo4};

void setup() {
  // put your setup code here, to run once:
pinMode(3, OUTPUT);
pinMode(5, OUTPUT);
pinMode(6, OUTPUT);
pinMode(9, OUTPUT);
pinMode(A0, INPUT);
Serial.begin(9600);
Serial.setTimeout(10);
while(!Serial);
while (Serial.available()>0){
  Serial.read();
}

for (int i = 0; i<4; i++) {
  servos[i].attach(pins[i]);
}
}


void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0) {
    val = Serial.readString();

    //Initialise values array to concatenate data
    String values[4] = {"", "", "", ""};

    //Run loop to concatenate data one at a time
    for (int i = 0; i<12; i++) {
      values[i/3] += val.charAt(i);
    }

    for (int i = 0; i<4; i++) {
      Serial.println(values[i]);
    }

    for (int i = 0; i<3; i++) {
      if (values[i] != "" && values[i].toInt()>=0 && values[i].toInt()<=180) {
          servos[i].write(values[i].toInt());
      }
    }
  }
  servos[3].write(map(analogRead(A0), 0, 1023, 0, 140));
}
