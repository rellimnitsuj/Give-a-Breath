//This is the script for the oxygen sensor

#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads1115;
int16_t O2;

void setup() {
  Wire.begin();
  ads1115.begin();
  Serial.begin(9600);
}

void loop() {
   
  O2 = ads1115.readADC_Differential_0_1();
  Serial.print("% O2: ");
  Serial.println((O2*.188)/.458851675);
  delay(500);
}
