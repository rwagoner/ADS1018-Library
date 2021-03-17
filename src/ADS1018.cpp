/**
*  Arduino Library for Texas Instruments ADS1018 - 12-Bit Analog-to-Digital Converter with 
*  Internal Reference and Temperature Sensor
*  
*  @author Ryan Wagoner <rswagoner@gmail.com>
*  @author Vishnu Easwaran E <easwaranvishnu@gmail.com>
*  derived from the work of Alvaro Salazar <alvaro@denkitronik.com>
*
*/

/**
 * The MIT License
 *
 * Copyright 2018 Vishnu Easwaran E <easwaranvishnu@gmail.com>
 * Copyright 2020 Ryan Wagoner <rswagoner@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "ADS1018.h"
#include <Arduino.h>

//#define DEBUG_ADS1018_CONFIG
//#define DEBUG_ADS1018_ADC

#ifdef DEBUG_ADS1018_CONFIG
    #define DEBUG_ADS1018_CONFIG(x) decodeConfigRegister(x)
#else
    #define DEBUG_ADS1018_CONFIG(x)
#endif

#ifdef DEBUG_ADS1018_ADC
    #define DEBUG_ADS1018_ADC(x) Serial.printf("ADC VALUE: %i\n", x);
#else
    #define DEBUG_ADS1018_ADC(x)
#endif

/*
 * Constructor of the class
 * @param io_pin_cs a byte indicating the pin to be use as the chip select pin (CS)
 */
ADS1018::ADS1018(uint8_t io_pin_cs) {
    cs = io_pin_cs;
}

/*
 * This method initialize the SPI port and the config register
 */
void ADS1018::begin() {
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    SPI.begin();
    configRegister.bits={RESERVED, VALID_CFG, PULLUP, ADC_MODE, RATE_1600SPS, SINGLE_SHOT, FSR_2048, DIFF_0_1, START_NOW}; //Default values
}

/*
 * Getting a sample from the specified input
 */
uint16_t ADS1018::getSingleValue(uint8_t inputs) {
    uint16_t convRegister;
    union Config sendConfigRegister;
    
    sendConfigRegister.word = configRegister.word;
    sendConfigRegister.bits.mux=inputs;
    sendConfigRegister.bits.sensorMode=ADC_MODE;
    sendConfigRegister.bits.operatingMode=SINGLE_SHOT;
    
    convRegister = readSingle(sendConfigRegister);

    return convRegister;
}

/*
 * Getting the voltage in V from the specified input
 */
double ADS1018::getSingleVoltage(uint8_t inputs) {   
    uint16_t value;
    value = getSingleValue(inputs);   
    return convertToVoltage(value);
}

/*
 * Getting the voltage in V from the given sample
 */
double ADS1018::convertToVoltage(uint16_t value) {   
    return value * PGA_FSR[configRegister.bits.pga] / 1000;
}

/*
 * Getting the temperature in degrees celsius from the internal sensor
 * @return A double (32bits) containing the temperature in degrees celsius
 */
double ADS1018::getSingleTemperature() {
    uint16_t convRegister;
    union Config sendConfigRegister;
    
    sendConfigRegister.word = configRegister.word;
    
    sendConfigRegister.bits.mux=AIN_0;
    sendConfigRegister.bits.sensorMode=TEMP_MODE;
    sendConfigRegister.bits.operatingMode=SINGLE_SHOT;

    convRegister = readSingle(sendConfigRegister);

    if((convRegister) >= 0x0800) {
        convRegister=((~convRegister)+1 & 0x0fff); //Applying binary twos complement format
        return (double)convRegister*0.125;
    }
    
    return (double)convRegister*0.125;
}

/* 
 * Send config and get conversion register from the specified input
 */
uint16_t ADS1018::readSingle(union Config config) {
    uint16_t convRegister;
    uint8_t  dataMSB, dataLSB;
    
    SPI.beginTransaction(SPISettings(SCLK, MSBFIRST, SPI_MODE1));
    
    /* Using the 32-bit data transmission cycle
       Works without CS pin if CS is held low
       Refer to page 18 of ADS1018 */
    
    DEBUG_ADS1018_CONFIG(config);
    
    // Send config
    digitalWrite(cs, LOW);
    delayMicroseconds(10);
    SPI.transfer(config.byte.msb);
    SPI.transfer(config.byte.lsb);
    config.byte.msb = SPI.transfer(0);
    config.byte.lsb = SPI.transfer(0);
    
    delay(CONV_TIME[config.bits.rate]);
    
    // Read result  
    dataMSB = SPI.transfer(0);
    dataLSB = SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    digitalWrite(cs, HIGH);
    delayMicroseconds(10);
    
    SPI.endTransaction();
    
    //DEBUG_ADS1018_CONFIG(config);
    
    convRegister  = (((dataMSB)<<8 | (dataLSB)) >> 4); //Moving MSB and LSB to 16 bit and making it right-justified; 4 because 12bit value
    convRegister &= 0x0FFF; //Making sure first 4 bits are 0
    
    DEBUG_ADS1018_ADC(convRegister);
    
    return convRegister;
}

/*
 * Setting the full scale range in the config register
 * @param fsr The full scale range: FSR_6144 (±6.144V)*, FSR_4096(±4.096V)*, FSR_2048(±2.048V), FSR_1024(±1.024V), FSR_0512(±0.512V), FSR_0256(±0.256V). (*) No more than VDD + 0.3 V must be applied to this device.
 */
void ADS1018::setFullScaleRange(uint8_t fsr){
    configRegister.bits.pga=fsr;
}

/*
 * Setting the sampling rate specified in the config register
 * @param samplingRate It's the sampling rate: RATE_8SPS, RATE_16SPS, RATE_32SPS, RATE_64SPS, RATE_128SPS, RATE_250SPS, RATE_475SPS, RATE_860SPS
 */
void ADS1018::setSamplingRate(uint8_t samplingRate){
    configRegister.bits.rate=samplingRate;
}

/*
 * Disabling the internal pull-up resistor of the DOUT pin
 */
void ADS1018::disablePullup(){
    configRegister.bits.operatingMode=NO_PULLUP;
}

/*
 * Enabling the internal pull-up resistor of the DOUT pin
 */
void ADS1018::enablePullup(){
    configRegister.bits.operatingMode=PULLUP;
}

/*
 * Decode the configRegister structure and print it to the Serial port
 * @param configRegister The config register in "union Config" format
 */
void ADS1018::decodeConfigRegister(union Config config){
    String message=String();
    switch(config.bits.singleStart){
        case 0: message="NOINI"; break;
        case 1: message="START"; break;
    }
    message+=" ";
    switch(config.bits.mux){
        case 0: message+="A0-A1"; break;
        case 1: message+="A0-A3"; break;
        case 2: message+="A1-A3"; break;
        case 3: message+="A2-A3"; break;
        case 4: message+="A0-GD"; break;
        case 5: message+="A1-GD"; break;
        case 6: message+="A2-GD"; break;
        case 7: message+="A3-GD"; break;
        }
    message+=" ";
    switch(config.bits.pga){
        case 0: message+="6.144"; break;
        case 1: message+="4.096"; break;
        case 2: message+="2.048"; break;
        case 3: message+="1.024"; break;
        case 4: message+="0.512"; break;
        case 5: message+="0.256"; break;
        case 6: message+="0.256"; break;
        case 7: message+="0.256"; break;
        }
    message+=" ";       
    switch(config.bits.operatingMode){
        case 0: message+="CONT."; break;
        case 1: message+="SSHOT"; break;
    }
    message+=" ";       
    switch(config.bits.rate){
        case 0: message+="128SPS"; break;
        case 1: message+="250SPS"; break;
        case 2: message+="490SPS"; break;
        case 3: message+="920SPS"; break;
        case 4: message+="1600SP"; break;
        case 5: message+="2400SP"; break;
        case 6: message+="3300SP"; break;
    }
    message+=" ";       
    switch(config.bits.sensorMode){
        case 0: message+="ADC_M"; break;
        case 1: message+="TMP_M"; break;
    }
    message+=" ";       
    switch(config.bits.pullUp){
        case 0: message+="DISAB"; break;
        case 1: message+="ENABL"; break;
    }
    message+=" ";       
    switch(config.bits.noOperation){
        case 0: message+="INVAL"; break;
        case 1: message+="VALID"; break;
        case 2: message+="INVAL"; break;
        case 3: message+="INVAL"; break;
    }
    message+=" ";       
    switch(config.bits.reserved){
        case 0: message+="RSRV0"; break;
        case 1: message+="RSRV1"; break;
    }   
    Serial.println("START MXSEL PGASL MODES RATES  ADTMP PLLUP NOOPE RESER");
    Serial.println(message);
}
