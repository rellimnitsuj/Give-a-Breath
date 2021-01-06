//This is the script for modulating the temp of the heater 
int Heater = 3;

void setup()
{
  Serial.begin(9600);
}

void loop(){ 
  analogWrite(3,255/2); //0-255
  delay(1000);
}
