//This is the basic calibration function.
//Current: V0.1: 1 - point (20.90%) air calibration. 
//To Do: EEPROM read/write.

#include <Adafruit_ADS1X15.h>
#include <TimerOne.h>
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

float result, pts[] = {20.90, 12, 100, 57}; //x1 y1 x2 y2
int16_t adc0; //ADC Value
int16_t mv = 0; //equivalent milliVolts of the input.
volatile unsigned int counter = 0;
volatile bool pressed_flag = 0; //if calibration mode has to be started.
volatile bool pin_flag = 0;
bool calibrated = true;
//ADC object.
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
  // age array is passed to calculateSum()
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.begin();
  ads.startComparator_SingleEnded(0, 1000);
  //Calibration
  pinMode (CALIBRATION_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CALIBRATION_PIN), pin_isr, FALLING); // ext hardware interrupt (LOW)
  Timer1.attachInterrupt(counter_isr); // blinkLED to run every 0.15 seconds
  solveEqn(); //to get the initial values.


}


void loop () {


  if (pressed_flag) {
    //means to enter calibraiton mode;
    pressed_flag = 0; //reset flag.
    Timer1.stop(); //stop the timer.
    calibrationRoutine();//enter calibration routine.
  }

  if (calibrated) {
    float calibrated_purity = (ads.getLastConversionResults() * RESOLUTION_MV - c_new) / m_new; // x= (y-c)/m
    Serial.print("O2 Purity_CAL \t"); Serial.println(calibrated_purity, 2);
  }

  else {
    float general_purity = (ads.getLastConversionResults() * RESOLUTION_MV - c) / m; // x= (y-c)/m
    Serial.print("O2 Purity_CAL \t"); Serial.println(general_purity, 2);
    delay(1000);
  }

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

  //update new ADC voltage at 20.90 % air :
  //pts[1] = new_value;
  calibrated = true;
  //calculate the new parameters:
  solveEqn();


}

void solveEqn() {
  calibrated = 0;
  if (calibrated) {
    //if it has been calibrated, update the 2 points. 
    m_new =  (pts[3] - new_value) / (pts[2] - pts[0]) ; //slope is y2-y1/ x2-x1
    c_new =  new_value - ((m_new) * pts[0]); // intercept is y-mx
  }

  else {
    m =  (pts[3] - pts[1]) / (pts[2] - pts[0]) ; //slope m is y2-y1/ x2-x1
    c =  pts[1] - (m) * pts[0]; // intercept c = y-mx
    Serial.println(m, 4); Serial.println(c, 4);
  }
}


