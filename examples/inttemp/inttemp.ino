#include <ADS1018.h>

/**
* Internal Temp Example for using the Arduino Library for the Texas Instruments ADS1018 -
* a 12-Bit Analog-to-Digital Converter with Internal Reference and Temperature Sensor
*/

// Definition of the pin to be used as the chip select pin (SPI CS pin). Example: pin 5
#define CS 17

// Creating an ADS1018 object (object's name is ads1018)
ADS1018 ads1018(CS);

void setup(){
    Serial.begin(115200);

    // Initialize the ADS1018
    ads1018.begin();
}

void loop(){
    Serial.printf("Int Temp: %0.2f C\n", ads1018.getSingleTemperature());
    delay(500);
}
