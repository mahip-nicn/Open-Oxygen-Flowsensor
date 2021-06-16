/*
 *This is the basic calibration function for the open-source oxygen purity analyzer with flow sensor.
 *Current: V0.1: 2-point factory calibration (21% and 100%) and 1-point on-field (20.90%) offset calibration
 *IMPORTANT: The voltage value for 100% pure oxygen has to be set in the pts[] array (factory calibration). 
 *This value may be different for every other sensor so must be calibrated with reference to a known oxygen purity(medical oxygen cylinders). 
 *
 */

#include <Adafruit_ADS1X15.h>
#include <TimerOne.h>
#include <EEPROM.h>
#define CALIBRATION_PIN 2 // pin 2 has to be held for 5 seconds to enter calibration mode. 
#define RESOLUTION_MV 0.1875 //resolution for FSR ~ 6.2V. TODO: FSR reduce to get more resoltion. Input voltage is <100mV from sensor. 
//variables.
float m = 0;
float c = 0;
float m_new = 0;
float c_new = 0;
float new_value = 0;
void solveEqn();
void calibrationRoutine();

float result, pts[] = {20.90, 12, 100, 57}; //x1 y1 x2 y2;
int16_t adc0; //ADC Value
int16_t mv = 0; //equivalent milliVolts of the input.
volatile unsigned int counter = 0;
volatile bool pressed_flag = 0; //if calibration mode has to be started.
volatile bool pin_flag = 0;
//ADC object.
//EEPROM Values:
#define  airAddress  0 //always write to 21% air's value address.
#define  calAddress 10 //always write calibration on 11th address, far from air EEPROM data. 
float airContainer = 0; //for holding the write value.
bool calContainer = false; //to hold the previous calibration state
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

void counter_isr() {

  //Do timer counting here.
  if (!digitalRead(CALIBRATION_PIN)) {
    counter++;
    if (counter > 9) {
      counter = 0;
      pressed_flag = true; //5 seconds elapsed.
    }
  }
}

void pin_isr() {

  Timer1.start(); //start the counter.
  Serial.println("Calibration Button pressed");
  Serial.println("Hold for 5 s to enter calibration");
}

void setup() {

  Serial.begin(9600);
  Timer1.initialize(500000); //500ms period
  //Set the EEPROM and calibration points:
  EEPROM.get(calAddress, calContainer);
  delayMicroseconds(5);
  if (calContainer) {
    //means been calibrated
    //fetch last stored value and create the array.
    EEPROM.get(airAddress, airContainer);
    delayMicroseconds(5);
    Serial.print("Got stored value as:\t"); Serial.println(airContainer, 4);
    //retreive the saved value.
    pts[1] = airContainer;
  }

  // age array is passed to calculateSum()
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.begin();
  ads.startComparator_SingleEnded(0, 1000);
  //Calibration
  pinMode (CALIBRATION_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CALIBRATION_PIN), pin_isr, FALLING); // ext hardware interrupt (LOW)
  Timer1.attachInterrupt(counter_isr); // blinkLED to run every 0.15 seconds
  solveEqn(); //to get the initial values.
  //remove this for deployment
//  delay(1000);

}


void loop () {


  if (pressed_flag) {
    //means to enter calibraiton mode;
    pressed_flag = 0; //reset flag.
    Timer1.stop(); //stop the timer.
    calibrationRoutine();//enter calibration routine.
  }

  if (calContainer) {
    float calibrated_purity = (ads.getLastConversionResults() * RESOLUTION_MV - c_new) / m_new; // x= (y-c)/m
    //    Serial.println(ads.getLastConversionResults());
    Serial.print("O2 Purity_CAL \t"); Serial.println(calibrated_purity, 2);
  }

  else {
    float general_purity = (ads.getLastConversionResults() * RESOLUTION_MV - c) / m; // x= (y-c)/m
    Serial.print("O2 Purity_GENERAL \t"); Serial.println(general_purity, 2);
  }
  //get the updated values:
  solveEqn();
  //remove this for deployment
//  delay(1000);
//  Serial.println(airContainer);
//  Serial.println(calContainer);
}
//  solveEqn(pts, &m, &c);


void calibrationRoutine() {
  Serial.println("Starting Calibration routine...");
  Serial.println("Remove gasses from the sensor");
 
  //get new values:
  int16_t sum = 0;
  for (int i = 0; i < 10; i++) {
    int16_t temp_value = ads.getLastConversionResults();
    sum += temp_value;
    delayMicroseconds(20);
  }
  new_value = (sum / 10) * RESOLUTION_MV ;
  Serial.print("New value \t"); Serial.println(new_value, 4);
  EEPROM.put(airAddress, new_value);
  delayMicroseconds(5);
  EEPROM.get(airAddress, airContainer);
  delayMicroseconds(5);
  Serial.print("Written the new calibration value in EEPROM:\t"); Serial.println(airContainer);
  delay(500);
  //update new ADC voltage at 20.90 % air :
  pts[1] = new_value; //store this value. 
  new_value=0; 
  if (!calContainer) {
    //of the previous value is not true,
    EEPROM.put(calAddress, true);
    calContainer = true; //set this for using it on the rest of the code .
  }

  //calculate the new parameters:
  solveEqn();


}

void solveEqn() {
  //  for (int i = 0; i < 4; i++) {
  //    Serial.println(pts[i]);
  //  }
  
  if (calContainer) {
    //if it has been calibrated, update the 2 points.
    m_new =  (pts[3] - pts[1]) / (pts[2] - pts[0]) ; //slope is y2-y1/ x2-x1
    c_new =  pts[1] - ((m_new) * pts[0]); // intercept is y-mx
    //    Serial.println(m_new, 4); Serial.println(c_new, 4);

  }

  else {
    m =  (pts[3] - pts[1]) / (pts[2] - pts[0]) ; //slope m is y2-y1/ x2-x1
    c =  pts[1] - (m) * pts[0]; // intercept c = y-mx
    //    Serial.println(m, 4); Serial.println(c, 4);

  }

}
