//This is the script for controlling the blower 

int RPWM = 5; 
int R_EN = 6; 
int LPWM = 7; 
int L_EN = 8; 

void setup()
{
  Serial.begin (9600);
  pinMode(LPWM, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);
}

void loop()
{
  digitalWrite(LPWM, LOW);
  analogWrite(RPWM,0); 
  delay(3000);
  analogWrite(RPWM,255);
  delay(3000);
}
