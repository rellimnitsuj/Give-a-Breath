#include <PID_v1.h>
#include <Wire.h>
#include <hsc_ssc_i2c.h>
#include <Adafruit_ADS1015.h>
#include <HIH8121.h>
#include <Nextion.h>
#include <SFM3X00.h>

#define SLAVE_ADDR 0x58
#define OUTPUT_MIN 666
#define OUTPUT_MAX 0x399A
#define PRESSURE_MIN 0.0
#define PRESSURE_MAX 689475.72931783
#define SAMPLE_DELAY 550
#define FLOW_SENSOR_ADDRESS 0x40
#define T_BUFFER 500
#define IN_BUFFER 100
#define O2_BUFFER 5
#define P_BUFFER 2


/*Constants and Variables */
//pin assignments
//int _o2Valve = 2;
int _chamber = 3;
int _alarm = 4;
int _ambValve = 2;

//system variables [preallocated with default values]
float FiO2_target = 21;                           //set target O2 concentration (%)
float FiO2_max = 100;                             //set maximum allowable O2 concentration (%)
float FiO2_min = 21;                              //set minimum allowable O2 concentration (%)
float FiO2;                                       //store current FiO2 reading (%)
boolean alarm_FiO2 = false;                       //set FiO2 alarm

float V_target = 500;                             //set target Volume (mL)
float V_max = 2000;                               //set maximum allowable volume (mL)
float V_min = 0;                                  //set minimum allowable volume (mL)
float V;                                          //store current volume (mL)
boolean alarm_V = false;                          //set volume alarm

float P_target = 25;                              //set target inspiratory pressure (cmH2O)
float P_max = 35;                                 //set maximum allowable inspiratory pressure (cmH2O)
float P_min = 20;                                 //set minimum allowable inspiratory pressure (cmH2O)
float Peep_target = 5;                            //set target expiratory Pressure (cmH2O)
float Peep_min = 5;                               //set minimum allowable expiratory pressure (cmH2O)
float P;                                          //store current pressure (cmH2O)
boolean alarm_P = false;                          //set pressure (insp) alarm
boolean alarm_Peep = false;                       //set pressure (exp) alarm

float Q;                                          //store current flowrate (L/min)

float T_target = 34;                              //set target temperature (C)
float T;                                          //store current temperature reading (C) (initialized to lower bound room temp)

float H;                                          //store current humidity reading (%) (initialized to lower bound room temp humidity)

float BPM = 20;                                   //set breaths per minute
float insp = 1;                                   //set relative inspiratory time (inspiratory:expiratory ratio)
float expir = 2;                                  //set relative expiratory time (inspiratory:expiratory ratio)

int vMode = 0;                                    //store venhilator mode (0:CSV, 1: ASB, 2: Mandatory)

//Timing variables
float t = 0;                                      //store relative current time (ms)
float t0 = 0;                                     //store absolute current time (ms)
float minStart = 0;                               //store start time of current minute (ms)
float minEnd = 0;                                 //store end time of current minute (ms)
float bStart = 0;                                 //store start time of current breath (ms)
float bEnd = 0;                                   //store end time of current breath (ms)
float t_assist = 0;                               //store start time of assisted inhale (ms)
float t_breath = 60000 / BPM;                     //calculate time of breath (ms)
float partsWhole = insp + expir;                  //calculate total number of parts in ratio
float t_inhale = t_breath * (insp / partsWhole);  //calculate duration of inhalation (ms)
float t_exhale = t_breath * (expir / partsWhole); //calculate duration of expiration (ms)
float t_fs = 0;                                   //store current time of when flow sensor was last read (ms)

boolean vent = false;                             //store system ventilation status (T/F)

//PID variable
double tempSet, tempIn, tempOut;
PID tempPID(&tempIn, &tempOut, &tempSet, 2, 5, 1, DIRECT);

//Sensor Variables
uint32_t prev = 0;
const uint32_t interval = 1000;
int16_t O2;
Adafruit_ADS1115 ads1115;
byte address = 0x27;
HIH8121 hih8121(address);                         //create an instance of temp/humidity sensor with address
SFM3X00 flowSensor(FLOW_SENSOR_ADDRESS);          //create insance of sensor with address

//Nextion Object Variables
NexButton power = NexButton(1, 5, "b0");
NexButton snooze = NexButton(1,7,"b1");

NexSlider tPress = NexSlider(3, 2, "h0");
NexSlider tVol = NexSlider(3, 3, "h2");
NexSlider tBPM = NexSlider(3, 4, "h5");
NexSlider tInspExp = NexSlider(3, 5, "h6");
NexSlider tMode = NexSlider(3, 30, "h1");
NexSlider tO2 = NexSlider(3, 19, "h8");
NexButton decEP = NexButton(3, 24, "b0");
NexButton incEP = NexButton(3, 25, "b1");
NexVariable EP = NexVariable(3, 27, "ep");

NexSlider pressMin = NexSlider(4,3,"h0");
NexSlider pressMax = NexSlider(4,7,"h1");
NexSlider volMin = NexSlider(4,11,"h2");
NexVariable vMinVar = NexVariable(4,17,"vMin");
NexSlider volMax = NexSlider(4,14,"h3");
NexVariable vMaxVar = NexVariable(4,18,"vMax");
NexSlider oMin = NexSlider(4,19,"h4");
NexSlider oMax = NexSlider(4,22,"h5");
NexButton pAlrm = NexButton(4, 27, "b0");
NexButton vAlrm = NexButton(4, 28, "b1");
NexButton oAlrm = NexButton(4, 29, "b2");
NexButton peepAlrm = NexButton(4, 34, "b3");

//List of GUI Events
NexTouch *nex_listen_list[] =
{
  &power,                                             //button added
  &snooze,                                            //button added

  &tPress,                                            //slider added
  &tVol,                                              //slider added
  &tBPM,                                              //slider added
  &tInspExp,                                          //slider added
  &tMode,                                             //slider added
  &tO2,                                               //slider added
  &decEP,                                             //button added
  &incEP,                                             //button added

  &pressMin,                                          //slider added
  &pressMax,                                          //slider added
  &volMin,                                            //slider added
  &volMax,                                            //slider added
  &oMin,                                              //slider added
  &oMax,                                              //slider added
  &pAlrm,                                             //button added
  &vAlrm,                                             //button added
  &oAlrm,                                             //button added
  &peepAlrm,                                          //button added
  
  NULL                                                //String terminated
};

/* Primary System Functions */
void setup() {
  Wire.begin();

  //Initialize GUI, sensors, and alarm
  initGUI();
  initFiO2();
  initPressureFlow();
  initTempHumid();
  initAlarm();

  //Instances of GUI communication 
  power.attachPop(powerPopCallback, &power);
  snooze.attachPush(snoozePushCallback, &snooze);
  tPress.attachPop(tPressPopCallback, &tPress);
  tVol.attachPop(tVolPopCallback, &tVol);
  tBPM.attachPop(tBPMPopCallback, &tBPM);
  tInspExp.attachPop(tInspExpPopCallback, &tInspExp);
  tMode.attachPop(tModePopCallback, &tMode);
  tO2.attachPop(tO2PopCallback, &tO2);
  decEP.attachPop(decEPPopCallback, &decEP);
  incEP.attachPop(incEPPopCallback, &incEP);
  pressMin.attachPop(pressMinPopCallback, &pressMin);
  pressMax.attachPop(pressMaxPopCallback, &pressMax);
  volMin.attachPop(volMinPopCallback, &volMin);
  volMax.attachPop(volMaxPopCallback, &volMax);
  oMin.attachPop(oMinPopCallback, &oMin);
  oMax.attachPop(oMaxPopCallback, &oMax);
  pAlrm.attachPop(pAlrmPopCallback, &pAlrm);
  vAlrm.attachPop(vAlrmPopCallback, &vAlrm);
  oAlrm.attachPop(oAlrmPopCallback, &oAlrm);
  peepAlrm.attachPop(peepAlrmPopCallback, &peepAlrm);
}

void loop() {
  //Continually update GUI if ventilation is off
  if (!vent) {
    updateIn();
    writeGUI();
  //Run continuous spontaneous ventilation if selected
  } else if (vMode == 0) {
    //Wait for patient to inhale
    while (P >= -3) {
      setTime2();
      updateIn();
      writeGUI();

      if (T < T_target) {
        tempIn = (double)T;
        tempPID.Compute();
        heat((int)tempOut);
      } else {
        heat(0);
      }

      if (!vent) {
        shutDown();
        break;
      }
    }

    if (vent) {
      //Reset flow rate and inspiratory volume here to allow user time to read values
      Q = 0;
      V = 0;
      
      //Begin inhalation
      if (FiO2_target <= 90) {
        ambValveOn();
        bStart = t0;
        bEnd = bStart + t_breath;
        t = t0 - bStart;
      }
//      if (FiO2_target >= 21) {
//        o2ValveOn();
//      }
  
      //During inhalation, update GUI and check FiO2
      while (t <= t_inhale) {
        setTime2();
  
        nexLoop(nex_listen_list);
        updateIn();
        writeGUI();
        
        if (alarm_FiO2) {
          checkMinO2();
          checkMaxO2();
        }
      }
  
      //Check inspiratory pressure and volume at peak of inhalation
      if (alarm_P) {
        checkMaxP();
        checkMinP();
      }
      if (alarm_V) {
        checkMaxV();
      }
  
      //Shut down ventilation if critical
      if (!vent) {
        shutDown();
      }
  
      //Begin Exhalation
      ambValveOff();
//      o2ValveOff();
    }
  //Run assisted ventilation if selected
  } else if (vMode == 1) {
    setTime2();
    while (t > t_inhale) {
      setTime2();
    }

    //Reset flow rate and inspiratory volume here to allow user time to read values
    Q = 0;
    V = 0;
    
    //Begin inhalation
    if (FiO2_target <= 90) {
      ambValveOn();
    }
//    if (FiO2_target >= 21) {
//      o2ValveOn();
//    }

    //During inhalation, update GUI and check FiO2
    while (t <= t_inhale) {
      setTime2();

      nexLoop(nex_listen_list);
      updateIn();
      writeGUI();

      if (alarm_FiO2) {
        checkMinO2();
        checkMaxO2();
      }
      
      setTime2();
    }

    //Check inspiratory pressure and volume at peak of inhalation
    if (alarm_P) {
      checkMaxP();
      checkMinP();
    }
    if (alarm_V) {
      checkMaxV();
    }

    //Shut down ventilation if critical
    if (!vent) {
      shutDown();
    }

    //Begin Exhalation
    ambValveOff();
//    o2ValveOff();

    //During exhalation, increase humidity (if necessary) and update GUI
    while (t > t_inhale - T_BUFFER) {
      setTime2();

      if (P < -3) {
        //Reset flow rate and inspiratory volume here to allow user time to read values
        Q = 0;
        V = 0;
        
        //Begin inhalation
        if (FiO2_target <= 90) {
          ambValveOn();
          bStart = t0;
          bEnd = bStart + t_breath;
          t = t0 - bStart;
        }
//        if (FiO2_target >= 21) {
//          o2ValveOn();
//        }
  
        while (t <= t_inhale) {
          setTime2();
          dbSerial.println(t);
        
          nexLoop(nex_listen_list);
          updateIn();
          writeGUI();

          if (alarm_FiO2) {
            checkMinO2();
            checkMaxO2();
          }
        }
//  
        //Check inspiratory pressure and volume at peak of inhalation
        if (alarm_P) {
          checkMaxP();
          checkMinP();
        }
        if (alarm_V) {
          checkMaxV();
        }
  
        //Shut down ventilation if critical
        if (!vent) {
          shutDown();
        }
  
        //Begin Exhalation
        ambValveOff();
//        o2ValveOff();
      } else {
        if (T < T_target) {
          tempIn = (double)T;
          tempPID.Compute();
          heat((int)tempOut);
        }
        else {
          heat(0);
        }
  
        nexLoop(nex_listen_list);
        updateIn();
        writeGUI();
  
        setTime2();
      }
    }

    //Check expiratory pressure at peak of exhalation
    if (alarm_Peep) {
      checkMinPeep();
    }

    //Shut down ventilation if critical
    if (!vent) {
      shutDown();
    }
  
  //Run mandatory ventilation if selected
  } else {
    setTime();
    for (int b = 0; b < BPM; b++) {
      //Buffer interval to allow proper inspiratory:expiratory ratio
      while (t < b * t_breath) {
        setTime();
      }

      //Reset flow rate and inspiratory volume here to allow user time to read values
      Q = 0;
      V = 0;
      
      //Begin inhalation
      if (FiO2_target <= 90) {
        ambValveOn();
      }
//      if (FiO2_target >= 21) {
//        o2ValveOn();
//      }

      //During inhalation, update GUI and check FiO2
      while (b * t_breath <= t && t < b * t_breath + t_inhale) {
        setTime();

        nexLoop(nex_listen_list);
        updateIn();
        writeGUI();

        if (alarm_FiO2) {
          checkMinO2();
          checkMaxO2();
        }
        
        setTime();
      }

      //Check inspiratory pressure and volume at peak of inhalation
      if (alarm_P) {
        checkMaxP();
        checkMinP();
      }
      if (alarm_V) {
        checkMaxV();
      }

      //Shut down ventilation if critical
      if (!vent) {
        shutDown();
        break;
      }

      //Check if ventilation mode switched
      if (vMode != 2){
        break;
      }

      //Begin Exhalation
      ambValveOff();
//      o2ValveOff();

      //During exhalation, increase humidity (if necessary) and update GUI
      while (b * t_breath + t_inhale <= t && t < b * t_breath + t_breath - T_BUFFER) {
        setTime();

        if (T < T_target) {
          tempIn = (double)T;
          tempPID.Compute();
          heat((int)tempOut);
        }
        else {
          heat(0);
        }

        nexLoop(nex_listen_list);
        updateIn();
        writeGUI();

        setTime();
      }

      //Check expiratory pressure at peak of exhalation
      if (alarm_Peep) {
        checkMinPeep();
      }

      //Shut down ventilation if critical
      if (!vent) {
        shutDown();
        break;
      }

      //Check if ventilation mode switched
      if (vMode != 2){
        break;
      }
    }
  }
}



/* Secondary System Functions */
void updateIn() {
  //write code to change input values based on user interaction
  //to be called when user changes values on gui
  nexLoop(nex_listen_list);

  setPressure();
  setFlowAndVolume();
  setFiO2();
  setTempAndHumidity();

  t_breath = 60000 / BPM;                     //calculate time of breath (ms)
  partsWhole = insp + expir;                  //calculate total number of parts in ratio
  t_inhale = t_breath * (insp / partsWhole);  //calculate duration of inhalation (ms)
  t_exhale = t_breath * (expir / partsWhole); //calculate duration of expiration (ms)
}

void initGUI() {
  //write code to set up GUI
  nexInit();
  nexSerial.print("page1.pwr.val=");
  nexSerial.print(0);
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.write(0xff);
}

void initFiO2() {
  //write code to set up oxygen concentration sensor
  ads1115.begin();
//  pinMode(_o2Valve, OUTPUT);
  pinMode(_ambValve, OUTPUT);
//  o2ValveOff();
}

void initPressureFlow() {
  //write code to initialize both pressure sensors
  //set up both static pressure and flowrate
  flowSensor.begin();
}

void initTempHumid() {
  //write code to set up temperature and humidity sensor
  hih8121.begin();
  tempIn = (double) T;
  tempSet = (double) T_target;
  tempPID.SetMode(AUTOMATIC);

  pinMode(_chamber, OUTPUT);
  heat(0);
}

void initAlarm() {
  pinMode(_alarm, OUTPUT);
  alarmOff();
}

void setTime() {
  // set timer t for one minute
  t0 = millis();
  if (t0 >= minEnd) {
    minStart = t0;
    minEnd = minStart + 60000;
  }
  t = t0 - minStart;
}

void setTime2() {
  // set timer t for one breath
  t0 = millis();
  if (t0 >= bEnd) {
    bStart = t0;
    bEnd = bStart + t_breath;
  }
  t = t0 - bStart;
}

void setPressure() {
  //to be called on inspiration and expiration
  //assign static pressure value to variable P
  unsigned long now = millis();
  struct cs_raw ps;
  char p_str[10], t_str[10];
  uint8_t el;
  float p, t;
  el = ps_get_raw(SLAVE_ADDR, &ps);
  if (el != 4) {
    ps_convert(ps, &p, &t, OUTPUT_MIN, OUTPUT_MAX, PRESSURE_MIN, PRESSURE_MAX);
    p = p / 98.0665 - 494.35;
    P = round(p);
  } else {
    P = 0;
  }
}

void setFlowAndVolume() {
  //assign flowrate value to variable Q (LPM)
  //calculate volume value to variable V (mL)

  float flow = flowSensor.readFlow();
  float t_local = fmod(t,t_breath);
  if (flow > 0 && t_local > t_fs) {
    Q = flow;
    V += ((t_local - t_fs) * Q) / 60;
    t_fs = t_local;
  } else {
    t_fs = 0;
  }
}

void setTempAndHumidity() {
  //assign temperature value to variable T
  hih8121.readSensor();
  T = hih8121.temperature_C;
  H = hih8121.humidity;
}

void setFiO2() {
  O2 = ads1115.readADC_Differential_0_1();
  FiO2 = 100*((float)O2*.188)/((11.3*100)/20.9);
}


void writeGUI() {
  //include code to write updates to the GUI
  //handle status and error

  //write current PRESSURE (cmH20) to GUI
  nexSerial.print("page2.n0.val=");
  nexSerial.print((int)P);
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.write(0xff);

  //write current FLOWRATE (LPM) to GUI
  nexSerial.print("page2.n1.val=");
  nexSerial.print((int)Q);
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.write(0xff);

  //write current TEMP (C) to GUI
  nexSerial.print("page2.n2.val=");
  nexSerial.print((int)T);
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.write(0xff);

  //write current HUMIDITY (%) to GUI
  nexSerial.print("page2.n3.val=");
  nexSerial.print((int)H);
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.write(0xff);

  //write current 02 (%) to GUI
  nexSerial.print("page2.n4.val=");
  nexSerial.print((int)FiO2);
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.write(0xff);

  //write current VOLUME (mL) to GUI
  nexSerial.print("page2.n5.val=");
  nexSerial.print((int)V);
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.write(0xff);
}

boolean checkMinO2() {
  if (FiO2 < FiO2_min - O2_BUFFER) {
    dbSerial.println("mino2");
    alarmOn();
  }
}

void checkMaxO2() {
  if (FiO2 > FiO2_max + O2_BUFFER) {
    dbSerial.println("maxo2");
    alarmOn();
  }
}

void checkMaxP() {
  if (P > P_max + (P_BUFFER + P*0.04)) {
    dbSerial.println("maxp");
    alarmOn();
    vent = false;
  }
}

void checkMinP() {
  if (P < P_min - (P_BUFFER + P*0.04)) {
    dbSerial.println("minp");
    alarmOn();
    vent = false;
  }
}

void checkMinPeep() {
  if (P < Peep_min - (P_BUFFER + P*0.04)) {
    dbSerial.println("peep");
    alarmOn();
    vent = false;
  }
}

void checkMaxV() {
  if (V > V_max) {
    dbSerial.println("maxv");
    alarmOn();
    vent = false;
  }
}

void checkMinV() {
  if (V < V_min) {
    dbSerial.println("minv");
    alarmOn();
    vent = false;
  }
}

void ambValveOn() {
  digitalWrite(_ambValve, HIGH);          //open ambient air valve
}

void ambValveOff() {
  digitalWrite(_ambValve, LOW);           //close ambient air valve
}

//void o2ValveOn() {
//  digitalWrite(_o2Valve, HIGH);          //open O2 valve
//}
//
//void o2ValveOff() {
//  digitalWrite(_o2Valve, LOW);           //close O2 valve
//}

void alarmOn() {
  digitalWrite(_alarm, LOW);            //power on buzzer
  nexSerial.print("page1.snze.val=1");
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  
  nexSerial.print("page1.b1.bco=34335");
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.write(0xff);
}

void alarmOff() {
  digitalWrite(_alarm, HIGH);           //power off buzzer
}

void heat( int level) {
  analogWrite(_chamber, level);          //deliver appropriate power to heatng element
}

void shutDown() {
  ambValveOff();
//  o2ValveOff();
  heat(0);
  nexSerial.print("page1.pwr.val=0");
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.write(0xff);

  nexSerial.print("page1.b0.bco=63488");
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.write(0xff);

  nexSerial.print("page1.b0.txt=\"OFF\"");
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.write(0xff);

  t = 0;
//  t0 = 0;
  bStart = 0;
  bEnd = 0;
  minStart = 0;
  minEnd = 0;
  t_fs = 0;
}

void powerPopCallback(void *ptr) {
  dbSerial.println("power");
  if (vent == true) {
    vent = false;
  } else {
    vent = true;
  }
}

void snoozePushCallback(void *ptr) {
  alarmOff();
}

void tPressPopCallback(void *ptr) {
  uint32_t pNum = 0;
  tPress.getValue(&pNum);
  P_target = pNum;
}

void tVolPopCallback(void *ptr) {
  uint32_t vNum = 0;
  tVol.getValue(&vNum);
  V_target = vNum*50;
}

void tBPMPopCallback(void *ptr) {
  uint32_t bpmNum = 0;
  tBPM.getValue(&bpmNum);
  BPM = bpmNum;
}

void tInspExpPopCallback(void *ptr) {
  uint32_t ieNum = 0;
  tInspExp.getValue(&ieNum);
  expir = ieNum;
}

void tModePopCallback(void *ptr) {
  uint32_t mNum = 0;
  tMode.getValue(&mNum);
  vMode = mNum;
}

void tO2PopCallback(void *ptr) {
  uint32_t o2Num = 0;
  tO2.getValue(&o2Num);
  FiO2_target = o2Num;
}

void decEPPopCallback(void *ptr) {
  uint32_t decNum = 0;
  EP.getValue(&decNum);
  Peep_target = decNum;
}

void incEPPopCallback(void *ptr) {
  uint32_t incNum = 0;
  EP.getValue(&incNum);
  Peep_target = incNum;
}

void pressMinPopCallback(void *ptr) {
  uint32_t pMinNum = 0;
  pressMin.getValue(&pMinNum);
  P_min = pMinNum;
}

void pressMaxPopCallback(void *ptr) {
  uint32_t pMaxNum = 0;
  pressMax.getValue(&pMaxNum);
  P_max = pMaxNum;
}

void volMinPopCallback(void *ptr) {
  uint32_t vMinNum = 0;
  vMinVar.getValue(&vMinNum);
  V_min = vMinNum*50;
}

void volMaxPopCallback(void *ptr) {
  uint32_t vMaxNum = 0;
  vMaxVar.getValue(&vMaxNum);
  V_max = vMaxNum*50;
}

void oMinPopCallback(void *ptr) {
  uint32_t oMinNum = 0;
  oMin.getValue(&oMinNum);
  FiO2_min = oMinNum;
}

void oMaxPopCallback(void *ptr) {
  uint32_t oMaxNum = 0;
  oMax.getValue(&oMaxNum);
  FiO2_max = oMaxNum;
}

void pAlrmPopCallback(void *ptr) {
  if (alarm_P == true) {
    alarm_P = false;
  } else {
    alarm_P = true;
  }
}

void vAlrmPopCallback(void *ptr) {
  if (alarm_V == true) {
    alarm_V = false;
  } else {
    alarm_V = true;
  }
}

void oAlrmPopCallback(void *ptr) {
  if (alarm_FiO2 == true) {
    alarm_FiO2 = false;
  } else {
    alarm_FiO2 = true;
  }
}

void peepAlrmPopCallback(void *ptr) {
  if (alarm_Peep == true) {
    alarm_Peep = false;
  } else {
    alarm_Peep = true;
  }
}
