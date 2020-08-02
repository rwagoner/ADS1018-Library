#include <ADS1018.h>

/**
* Thermistor Example for using the Arduino Library for the Texas Instruments ADS1018 -
* a 12-Bit Analog-to-Digital Converter with Internal Reference and Temperature Sensor
*/

// Definition of the pin to be used as the chip select pin (SPI CS pin). Example: pin 5
#define CS 17

// Creating an ADS1018 object (object's name is ads1018)
ADS1018 ads1018(CS);

double Vcc = 3.3;       // Reference voltage
double R1 = 10000.0;    // Voltage divider resistor value
double Beta = 3977.0;   // Beta value
double To = 298.15;     // Temperature in Kelvin for 25 degree Celsius
double Ro = 10000.0;    // Resistance of Thermistor at 25 degree Celsius

void setup(){
    Serial.begin(115200);

    // Initialize the ADS1018
    ads1018.begin();
}

void loop(){
  double val, Vout, Rt = 0;
  double T, Tc, Tf = 0;
    
  val = ads1018.getSingleValue(ads1018.AIN_0);

  if(val == 2047) {
    Serial.println("Thermistor: not connected");
  } else if (val == 0) {
    Serial.println("Thermistor: shorted");
  } else {
    Vout = ads1018.convertToVoltage(val);
    Rt = R1 * Vout / (Vcc - Vout);
    T = 1/(1/To + log(Rt/Ro)/Beta);  // Temperature in Kelvin
    Tc = T - 273.15;                 // Celsius
    Tf = Tc * 9 / 5 + 32;            // Fahrenheit
    Serial.printf("Thermistor: %0.2fF\n", Tf);
  }
  
  delay(500);
}
