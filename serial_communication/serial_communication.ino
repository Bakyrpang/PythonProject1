void setup() {
  // put your setup code here, to run once:
pinMode(3, OUTPUT);
pinMode(5, OUTPUT);
pinMode(6, OUTPUT);
Serial.begin(9600);
Serial.setTimeout(10);
while(!Serial);
while (Serial.available()>0){
  Serial.read();
}
}

int pins[3] = {3,5,6};
String val;

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0) {
    val = Serial.readString();

    //Initialise values array to concatenate data
    String values[3] = {"", "", ""};

    //Run loop to concatenate data one at a time
    for (int i = 0; i<9; i++) {
      values[i/3] += val.charAt(i);
    }

    for (int i = 0; i<3; i++) {
      analogWrite(pins[i], values[i].toInt());
      Serial.println(values[i]);
    }
  }
}
