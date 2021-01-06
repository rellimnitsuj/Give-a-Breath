//This is the script for toggling the single valve on the ventilator for O2 flow
int Valve = 2;

void setup() {
  Serial.begin (9600);
}

void loop() {
  digitalWrite(Valve, LOW);
  delay(1000);
  digitalWrite(Valve, HIGH);
  delay(1000);
}
