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
 
#ifndef ADS1018_h
#define ADS1018_h

#include <Arduino.h>
#include <SPI.h>

/*
* Union representing the "config register" in 3 ways:
* bits, word (16 bits) and nibbles (4 bits)
* (See the datasheet [1] for more information)
*/

///Union configuration register
union Config {
    ///Structure of the config register of the ADS1018. (See datasheet [1])
    struct {
        uint8_t reserved:1;         ///< "Reserved" bit
        uint8_t noOperation:2;      ///< "NOP" bits
        uint8_t pullUp:1;           ///< "PULL_UP_EN" bit   
        uint8_t sensorMode:1;       ///< "TS_MODE" bit  
        uint8_t rate:3;             ///< "DR" bits
        uint8_t operatingMode:1;    ///< "MODE" bit     
        uint8_t pga:3;              ///< "PGA" bits
        uint8_t mux:3;              ///< "MUX" bits
        uint8_t singleStart:1;      ///< "SS" bit
    } bits;
    uint16_t word;                  ///< Representation in word (16-bits) format
    struct {
        uint8_t lsb;                ///< Byte LSB
        uint8_t msb;                ///< Byte MSB
    } byte;                         ///< Representation in bytes (8-bits) format
};

/*
 * Class representing the ADS1018 sensor chip
 */
class ADS1018 {
    public:
        ADS1018(uint8_t io_pin_cs);                                 ///< Constructor

        void     begin();                                           ///< This method initialize the SPI port and the config register
        
        uint16_t getSingleValue(uint8_t inputs);                    ///< Getting a sample from the specified input
        double   getSingleVoltage(uint8_t inputs);                  ///< Getting the voltage in V from the specified input
        double   convertToVoltage(uint16_t value);                  ///< Getting the voltage in V from the given sample
        double   getSingleTemperature();                            ///< Getting the temperature in degrees celsius from the internal sensor
        void     decodeConfigRegister(union Config configRegister); ///< Decode the configRegister structure and print it to the Serial port
        
        // Used by "FSR" bits
        void     setFullScaleRange(uint8_t fsr);                    ///< Setting the full scale range in the config register
        
        // Used by "DR" bits
        void     setSamplingRate(uint8_t samplingRate);             ///< Setting the sampling rate specified in the config register
        
        // Used by "PULL_UP_EN" bit
        void     disablePullup();                                   ///< Disabling the internal pull-up resistor of the DOUT pin
        void     enablePullup();                                    ///< Enabling the internal pull-up resistor of the DOUT pin
        
        union Config  configRegister;               ///< Config register

        // Bit constant
        const long int SCLK        = 1000000;       ///< ADS1018 SCLK frequency: 4000000 Hz Maximum for ADS1018 (4Mhz)

        // Used by "SS" bit
        const uint8_t START_NOW    = 1;             ///< Start of conversion in single-shot mode
        
        // Input multiplexer configuration used by "MUX" bits
        const uint8_t DIFF_0_1     = 0b000;         ///< Differential input: Vin=A0-A1
        const uint8_t DIFF_0_3     = 0b001;         ///< Differential input: Vin=A0-A3
        const uint8_t DIFF_1_3     = 0b010;         ///< Differential input: Vin=A1-A3
        const uint8_t DIFF_2_3     = 0b011;         ///< Differential input: Vin=A2-A3
        const uint8_t AIN_0        = 0b100;         ///< Single ended input: Vin=A0
        const uint8_t AIN_1        = 0b101;         ///< Single ended input: Vin=A1
        const uint8_t AIN_2        = 0b110;         ///< Single ended input: Vin=A2
        const uint8_t AIN_3        = 0b111;         ///< Single ended input: Vin=A3
        
        // Full scale range (FSR) selection by "PGA" bits
        const uint8_t FSR_6144     = 0b000;         ///< Range: ±6.144 v. LSB SIZE = 3mV
        const uint8_t FSR_4096     = 0b001;         ///< Range: ±4.096 v. LSB SIZE = 2mV
        const uint8_t FSR_2048     = 0b010;         ///< Range: ±2.048 v. LSB SIZE = 1mV ***DEFAULT
        const uint8_t FSR_1024     = 0b011;         ///< Range: ±1.024 v. LSB SIZE = 0.5mV
        const uint8_t FSR_0512     = 0b100;         ///< Range: ±0.512 v. LSB SIZE = 0.25mV
        const uint8_t FSR_0256     = 0b111;         ///< Range: ±0.256 v. LSB SIZE = 0.125mV
        
        // Used by "MODE" bit
        const uint8_t CONTINUOUS   = 0;             ///< Continuous conversion mode
        const uint8_t SINGLE_SHOT  = 1;             ///< Single-shot conversion and power down mode

        // Sampling rate selection by "DR" bits
        // Warning: this could increase the noise and the effective number of bits (ENOB)
        const uint8_t RATE_128SPS  = 0b000;         ///< 128 samples/s, Tconv=125ms
        const uint8_t RATE_250SPS  = 0b001;         ///< 250 samples/s, Tconv=62.5ms
        const uint8_t RATE_490SPS  = 0b010;         ///< 490 samples/s, Tconv=31.25ms
        const uint8_t RATE_920SPS  = 0b011;         ///< 920 samples/s, Tconv=15.625ms
        const uint8_t RATE_1600SPS = 0b100;         ///< 1600 samples/s, Tconv=7.8125ms ***DEFAULT
        const uint8_t RATE_2400SPS = 0b101;         ///< 2400 samples/s, Tconv=4ms
        const uint8_t RATE_3300SPS = 0b110;         ///< 3300 samples/s, Tconv=2.105ms
        
        // Used by "TS_MODE" bit
        const uint8_t ADC_MODE     = 0;             ///< External (inputs) voltage reading mode ***DEFAULT
        const uint8_t TEMP_MODE    = 1;             ///< Internal temperature sensor reading mode

        // Used by "PULL_UP_EN" bit
        const uint8_t INT_PULLUP   = 1;             ///< Internal pull-up resistor enabled for DOUT ***DEFAULT
        const uint8_t NO_PULLUP    = 0;             ///< Internal pull-up resistor disabled

        // Used by "NOP" bit
        const uint8_t VALID_CFG    = 0b01;          ///< Data will be written to Config register
        const uint8_t NO_VALID_CFG = 0b00;          ///< Data won't be written to Config register

        // Used by "Reserved" bit
        const uint8_t RESERVED     = 1;             ///< Its value is always 1, reserved

    private:
        uint16_t readSingle(union Config config);                   ///< Send config and get conversion register from the specified input
        
        uint8_t cs;                                 ///< Chip select pin (choose one)
        
        const float PGA_FSR[8] = {3, 2, 1, 0.5, 0.25, 0.125, 0.125, 0.125};
        const uint8_t CONV_TIME[8] = {125, 63, 32, 16, 8, 4, 3, 2};    ///< Array containing the conversions time in ms    
};

#endif