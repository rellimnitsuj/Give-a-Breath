//This is the script for the buzzer
int Buzzer = 4;

void setup() {
  Serial.begin (9600);
}

void loop() {
  digitalWrite(Buzzer, LOW);
  delay(1000);
}
