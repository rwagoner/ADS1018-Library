#include <ADS1018.h>

/**
* Singled Ended Example for using the Arduino Library for the Texas Instruments ADS1018 -
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
    
    // Set the desired ADC input range
    //ads1018.setFullScaleRange(ads1018.FSR_6144);    ///< Range: ±6.144 v. LSB SIZE = 3mV
    ads1018.setFullScaleRange(ads1018.FSR_4096);    ///< Range: ±4.096 v. LSB SIZE = 2mV
    //ads1018.setFullScaleRange(ads1018.FSR_2048);    ///< Range: ±2.048 v. LSB SIZE = 1mV ***DEFAULT
    //ads1018.setFullScaleRange(ads1018.FSR_1024);    ///< Range: ±1.024 v. LSB SIZE = 0.5mV
    //ads1018.setFullScaleRange(ads1018.FSR_0512);    ///< Range: ±0.512 v. LSB SIZE = 0.25mV
    //ads1018.setFullScaleRange(ads1018.FSR_0256);    ///< Range: ±0.256 v. LSB SIZE = 0.125mV
}

void loop(){
    int16_t adc0, adc1, adc2, adc3;
    
    adc0 = ads1018.getSingleValue(ads1018.AIN_0);
    adc1 = ads1018.getSingleValue(ads1018.AIN_1);
    adc2 = ads1018.getSingleValue(ads1018.AIN_2);
    adc3 = ads1018.getSingleValue(ads1018.AIN_3);
    
    Serial.printf("A0 Value:   %i\n", adc0);
    Serial.printf("A1 Value:   %i\n", adc1);
    Serial.printf("A2 Value:   %i\n", adc2);
    Serial.printf("A3 Value:   %i\n", adc3);
    
    Serial.printf("A0 Voltage: %0.2fV\n", ads1018.convertToVoltage(adc0));
    Serial.printf("A1 Voltage: %0.2fV\n", ads1018.convertToVoltage(adc1));
    Serial.printf("A2 Voltage: %0.2fV\n", ads1018.convertToVoltage(adc2));
    Serial.printf("A3 Voltage: %0.2fV\n", ads1018.convertToVoltage(adc3));

    delay(500);
}
