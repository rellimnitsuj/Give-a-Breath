//This the script for the differential pressure sensor

#include <Adafruit_ADS1015.h>   
Adafruit_ADS1115 ads1115(0x48);     
#include <Average.h>  
Average < float > averageValue(200);  
         
int sampleNumber = 0; // variable to store the sample number    
int16_t sensorValue = 0; // variable to store the Raw Data value coming from the sensor      
float averageInitialValue = 0; // variable to store the average inital value   
float diffPressure = 0; // variable to store converted kPa value    
float volumetricFlow = 0; // variable to store volumetric flow rate value    
float velocityFlow = 0; // variable to store velocity of flow value    
float offset = 0; // variable to store offset differential pressure      
const float tubeArea1 = 0.01455; // area of Large Part of Venturi Tube   
const float tubeArea2 = 0.0044; // area of Smaller Part of Venturi Tube   
const float airDensity = 1.206; // The density of air at 20 °C    
    
void setup() {     
  Serial.begin(115200);  
  ads1115.setGain(GAIN_ONE); // 1x gain  +/- 4.096V 1 bit = 2mV  
  ads1115.begin();  
   
  for (int i = 0; i < 200; i++) {  
   sensorValue = ads1115.readADC_SingleEnded(0);  
    delay(25);    
    averageValue.push(sensorValue);  
}  
  for (int i = 0; i < 200; i++) {  
    averageValue.get(i);      
}  
    averageInitialValue = averageValue.mean();   
}  
  
void loop() {  
   
sensorValue = ads1115.readADC_SingleEnded(2);  
sensorValue = sensorValue - (int) averageInitialValue;  
sampleNumber++;  
diffPressure = map(sensorValue, 0, 32767, 0, 4000);  

if (sensorValue >= 0) {  
//calculate volumetric flow rate for Exhalation   
volumetricFlow = tubeArea1 * (sqrt((2 / airDensity) * (diffPressure / (sq(tubeArea1 / tubeArea2) - 1))));  
//calculate velocity of flow    
velocityFlow = volumetricFlow / tubeArea1;  
}  
   
else if (sensorValue <= 0) {  
diffPressure = diffPressure * -1;  
//calculate volumetric flow rate for Inhalation   
volumetricFlow = tubeArea2 * (sqrt((2 / airDensity) * (diffPressure / (1 - sq(tubeArea2 / tubeArea1)))));  
//calculate velocity of flow    
velocityFlow = volumetricFlow / tubeArea2;  
}  

Serial.print(diffPressure);  
Serial.print(volumetricFlow);   
Serial.print(velocityFlow);  
Serial.println();  
delay(25);  

}  
