#include <HIH8121.h>
#include <Wire.h>

byte address = 0x27;
HIH8121 hih8121(address);

void setup() {
  Serial.begin(9600);
  hih8121.begin();
  Serial.println("Honeywell HIH8121 Humidity - Temperature Sensor");
  Serial.println("RH\tTemp (C)\tTemp (F)\tHeat Index (C)\tHeat Index (F)");
}

void loop() {
  hih8121.readSensor();
  Serial.print(hih8121.humidity); Serial.print("\t");
  Serial.print(hih8121.temperature_C); Serial.print("\t\t");
  Serial.print(hih8121.temperature_F); Serial.print("\t\t");
  Serial.print(hih8121.computeHeatIndex_C()); Serial.print("\t\t");
  Serial.println(hih8121.computeHeatIndex_F());
  delay(3000);
}
