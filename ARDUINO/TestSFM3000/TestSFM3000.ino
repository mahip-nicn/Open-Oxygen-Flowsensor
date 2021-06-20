//Bare minimum testing for SFM3000 Digital Flow Sensor.
//Tested on Arduino Nano.
#include <Wire.h>
#include <sfm3x00.h>

SFM3000 flowSensor;

const bool DEBUG = true;
float flow = 0;

void setup() {
  // Serial:

  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  while (true) {
    int ret_flowSensor = flowSensor.init();
    //Check if SFM has been initialized or not.
    if (!ret_flowSensor) {
      if (DEBUG) {
        Serial.println("Init complete!");
        break;
      }
      else {
        if (DEBUG) {
          Serial.println("Unable to init Flow Sensor");
          Serial.println(ret_flowSensor);
        }

      }

    }
  }
  flowSensor.get_scale_offset();
  flowSensor.start_continuous();
}

void loop () {

  if (ret_flowSensor == 0){
        flow = flowSensor.get_flow();
        Serial.print("Flow \t"); Serial.println(flow);
  }
    


}
