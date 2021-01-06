//This is the script for the static pressure sensor

#include <Wire.h>
#include <hsc_ssc_i2c.h>
#define SLAVE_ADDR 0x58
#define OUTPUT_MIN 666
#define OUTPUT_MAX 0x399A
#define PRESSURE_MIN 0.0
#define PRESSURE_MAX 689475.72931783

uint32_t prev = 0;
const uint32_t interval = 1000;

void setup() {
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
    unsigned long now = millis();
    struct cs_raw ps;
    char p_str[10], t_str[10];
    uint8_t el;
    float p, t;
        el = ps_get_raw(SLAVE_ADDR, &ps);
        if ( el == 4 ) {
            Serial.println("err sensor missing");
        } else {                      
            ps_convert(ps, &p, &t, OUTPUT_MIN, OUTPUT_MAX, PRESSURE_MIN,PRESSURE_MAX);
            p = p/6894.76  - 7;
            dtostrf(p, 2, 2, p_str);
            dtostrf(t, 2, 2, t_str);
            //Serial.print("Pressure (PSI) ");
            Serial.println(p_str);
    } 
}
