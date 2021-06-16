# Open Oxygen (O2) Flow Sensor

**WORK IN PROGRESS - more information will follow soon!**

This device is supposed to measure the oxygen flowrate of oxygen bottles.
We would like to have an open-source solution for something like the [TSI digital oxygen flow sensor](https://tsi.com/produkte/durchflussmesser,-durchflusssensoren-testsysteme/die-neuen-digitalen-durchflussmesser-serie-5000/).
<!-- 
<p align="center">
<a href="#logo" name="logo"><img src="./IMAGES/SETUP.jpeg" width="600"></a>
</p>
 -->
***Features:***

- Open-Source
- Low-Cost (~150€)
- Based on 3D parts + off-the-shelf components
- Minimum tools required
- Portable by relying on a USB power bank
- LCD for displying oxygen level (21%-100%) and flowrate

# Electronics

The flowsensor and the display are connected to the Arduino Nano via I2C (GPIO 21/22 for SDA/SCL), wheras the Oxygen Sensor is connected to the board via ADS1115 module. This module contains a 16 bit ADC with Programmable Gain Amplifier. 

<p align="center">
<a href="#logo" name="Wiring"><img src="./IMAGES/Components Overview.png" width="600"></a>
</p>


In order to make the device independent from external energy sources, we incorporate a USB-battery/powersupply. We rely on a widely available form factor, where the brand/name of the device can largely vary. 



# CAD Files

The parts were designed in Autodesk Inventor 2019 student edition and can be found in the folder [./CAD](CAD)
An enclosure/box to keep all parts in place will be 3D printable.

<p align="center">
<a href="#logo" name="Wiring"><img src="./IMAGES/Assemlby_OpenOxygen1.png" width="600"></a>
</p>

*Arrangement of all the parts*

## Housing for the setup

The setup will have a housing for all the parts. Currently it looks like this, where the battery is sitting on top:

<p align="center">
<a href="#logo" name="Wiring"><img src="./IMAGES/setup_housing.png" width="400"></a>
</p>

*Arrangement of all the parts*

<p align="center">
<a href="#logo" name="Wiring"><img src="./IMAGES/Housing.png" width="600"></a>
</p>

All **3D printable** files can be found in [./STL](./STL)


# Assembly

*Sorry for the confusion:* You can mount the parts in both directions. Here, the CAD rendering shows the oxygen sensor on the right hand side, whereas the printed version shows it on the right hand side. Either way works!

1. Make sure you solder all parts together according to the wiring diagram above; To keep the wire chaos small: Twist the cables to form wire bundles; Test its proper functionality by e.g. running the [I2C scanner in the Arduino IDE](https://playground.arduino.cc/Main/I2cScanner/) (you must detect 2 addresses); Then flash the firmware in the Arduino folder.  ***HINT:*** There are some libraries involved; Please install them from the library downloader menu as described [here](https://www.arduino.cc/en/guide/libraries). The following libraries are used: `LiquidCrystal_I2C by Brabander and Sparkfun SDP3X by Sparkfun`
2. Place the LCD screen on the front plate and mount it with 4 M3x12 screws 
3. Place the flow-rate sensor (sensirion) on the plate by pushing one end through one of the wholes. Watch the photograph:

<p align="center">
<a href="#logo" name="Wiring"><img src="./IMAGES/OpenOxy2.jpg" width="600"></a>
</p>

4. Now mount the S-connector to the outside of the injection moulded flow adapter (black plastic part of the sensirion); Beware: We don't mount the flow sensor permanently 
5. Mount the T-connector (tricky part, not much space); push the black flow adaptor at a side (e.g. diagonal) and try sneaking in the T-adaptor; Move the one thin end into the hole of the Flow sensor
6. Mount the Oxygen sensor to the open end of the T-connector inside the case 
7. Rotate the oxygen sensor the way that it resides on the base plate 
8. Place the ESP32 in the free spot on the base (no holes for screws, since the formfactor is largely varying!) 
9. Place the OPAMP in the assembly; make sure the open pin connects will **never** cause any shortcut!
10. Mount the LCD Screen using additional M3 screws:

<p align="center">
<a href="#logo" name="Wiring"><img src="./IMAGES/OpenOxy1.jpg" width="600"></a>
</p>

11. Add the Powerbank in the desired slot and also add the USB Micro cable
<p align="center">
<a href="#logo" name="Wiring"><img src="./IMAGES/OpenOxy3.jpg" width="600"></a>
</p>

12. Add the lid and mount it using M3 screws 
<p align="center">
<a href="#logo" name="Wiring"><img src="./IMAGES/OpenOxy4.jpg" width="600"></a>
</p>

13. You are done! If you want to have the assembly working under an anble, you can simply rotate the screen: 

<p align="center">
<a href="#logo" name="Wiring"><img src="./IMAGES/OpenOxy5.jpg" width="600"></a>
</p>


# Software

A first attempt to connect all components can be found in [./ARDUINO](./ARDUINO). This version uses Arduino Nano as the controlling unit. 

## Calibration


### Calibration of the air flow

The [datasheet](Sensirion_Differential_Pressure_AppNotes_EK-P4_Flow_Element.pdf) for the Sensirion EK-P4 Evaluation Kit
For SDP3x Flow sensor suggests a relationship between differential pressure and flow-rate measured in Standard Liters per Minute (slm). We fit a curve to the values given in the application note in order to have a parametrized function for the digital readout:


<p align="center">
<a href="#logo" name="Wiring"><img src="./PYTHON/Curve_Flow.png" width="600"></a>
</p>

The corresponding Python script to generate this figure can be found [here](PYTHON/Fit_Flow_Calibration_Function.py). 

The fit suggest the following function: 

```py
def func(x, a, b, c, d):
    return a+np.sqrt(b+x*d)*c
  
  
a=-20.04843438
b=59.52168936
c=3.11050553
d=10.35186327
```

which can easily be implemented on the ESP32. 





Use previously measured calibration data? 

### Calibration of the oxygen level

Factory(in-house)Calibration is a 2-point calibration described as follows:

1st point - 21% Oxygen, Room Air
2nd point - 100% Oxygen, Medical Oxygen Cylinders

_These points were cross-checked using a commercial oxygen analyzer._ [link](https://www.conrad.com/p/greisinger-gox-100-t-oxygen-meter-with-sensor-123101)

#### Calibration Procedure: 

- Based on the value generated by the ADC (ADS1115) at 21% and 100% oxygen, a line equation was formed in the slope-intercept form : y = mx + c.
- The inlet oxygen's purity was determined using this equation. 
- Data at intermittent values of oxygen purity was generated using [Pufferfish](https://www.pez-globo.org/) to compare the readings from the commercial oxygen analyzer and the calibration value obtained from the line equation

<p align="left">
<a href="#logo" name="Wiring"><img src="./IMAGES/Data2.png" width="600"></a>
</p>

<details>
<summary>Click here to see the data</summary>
    
| Pufferfish HFNC | OpenOXY Gain = 2/3 | GOX-100T | OpenOXY Gain = 16 |
|-----------------|--------------------|----------|-------------------|
| 22              | 20.09              | 21       | 21.56             |
| 30              | 31.45              | 28.4     | 29.25             |
| 40              | 36.72              | 37.5     | 38.7              |
| 50              | 47.2               | 46.5     | 47.71             |
| 60              | 57.81              | 55.6     | 58.25             |
| 70              | 68.36              | 64.3     | 66.38             |
| 80              | 73.63              | 72.6     | 75.6              |
| 90              | 84.18              | 81.7     | 84.62             |
| 100             | 94.73              | 90.3     | 94.07             |

</details>


#### On-Field Calibration: 
(In progress)
- This calibration will be a off-set calibration where only the value of 21% will be used to generate the intercept (offset).

# Bill of materials

- Misc Wires for connecting all parts (use proper soldered connection!)
- USB micro Cable for flashing the firmware and for supplying energy for portable-mode
- Various M3 Cylindrical Screws, M3x12mm, 1€ 
- [I2C LCD Display](https://de.aliexpress.com/item/32413056677.html?spm=a2g0o.productlist.0.0.37d82e0cBbmYmX&algo_pvid=b23eb3fa-26b7-4967-8927-370bceaf8f2b&algo_expid=b23eb3fa-26b7-4967-8927-370bceaf8f2b-0&btsid=0b0a050116225252291684896e9a75&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_), **1,82€**
- [ads1115 Analog Digital Converter, 16Bit, I2C Breakoutboard](https://de.aliexpress.com/item/32584944400.html?albpd=de32584944400&acnt=494-037-6276&aff_platform=aaf&albpg=539263010115&netw=u&albcp=12554800262&pvid=433db5a0-d519-414b-b0ec-d81757d569be&sk=UneMJZVf&scm=1007.23534.124000.0&trgt=539263010115&terminal_id=97219810e62b4073bd503e68a1993030&needSmbHouyi=false&albbt=Google_7_shopping&src=google&crea=de32584944400&aff_fcid=fb140952e0f349409d47e72bc9efb9c4-1623130168444-07808-UneMJZVf&gclid=Cj0KCQjwh_eFBhDZARIsALHjIKew4RrgVYxvZKy29i_h_szASqZX6TG-UfPUJ0vnS9FvnMyJQTIaZqoaAhbkEALw_wcB&albag=127990761348&aff_fsk=UneMJZVf&albch=shopping&albagn=888888&isSmbAutoCall=false&aff_trace_key=fb140952e0f349409d47e72bc9efb9c4-1623130168444-07808-UneMJZVf&rmsg=do_not_replacement&device=c&gclsrc=aw.ds) (Need to be tested); **2,32€**
- [Arduino Nano](https://www.banggood.com/Geekcreit-ATmega328P-Nano-V3-Controller-Board-Improved-Version-Module-Development-Board-p-940937.html?cur_warehouse=CN&ID=6300464&rmmds=buy), **9.00$**
- [oxygen sensor oxiplus a 00a101 / OOM202](https://de.aliexpress.com/item/4000133941125.html?spm=a2g0o.search0302.0.0.6e2032aa3UHNHZ&algo_pvid=5e64a15b-e5e3-4262-905a-03d4820550db&algo_expid=5e64a15b-e5e3-4262-905a-03d4820550db-3&btsid=0b0a050116225250680124211e9a75&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_); **50,01€**, [Datasheet](https://www.was-geisler.biz/WEBSERVER/Bausteine/Dateien/DB_AU-007_008.pdf)
- [SFM3000](https://www.mouser.com/ProductDetail/Sensirion/SFM3000-200C?qs=lAYGPh4Fp%252BzH9WcDkeu8Ww%3D%3D); **107$**, [Datasheet](https://www.mouser.com/datasheet/2/682/Sensirion_Mass_Flow_Meters_SFM3000_Datasheet-2001061.pdf)
- [USB Powerbank, 5V, 2200mAh (make sure, the battery looks like the one above!)](https://www.ebay.de/p/1437454225); **2,10€**, The brand can vary, but make sure it's fitting in the housing)


# Contribute
If you have a question or found an error, please file an issue! We are happy to improve the device!

# License
Please have a look into the dedicated License file.

# Disclaimer
We do not give any guarantee for the proposed setup. Please use it at your own risk. Keep in mind that Laser source can be very harmful to your eye and your environemnt! It is not supposed to be used as a medical device!
