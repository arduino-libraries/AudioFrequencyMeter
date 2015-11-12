/*
  Audio Frequencimeter library for Arduino Zero.
  Copyright (c) 2015 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
  
  
  Thanks to Amanda Ghassaei
  for the frequency detection algorithm posted on:
  
	http://www.instructables.com/id/Arduino-Frequency-Detection/
	Sept 2012
*/


#include "Arduino.h"
#include "wiring_private.h"

#pragma once

#define ARRAY_DEPTH 				20
#define AMPLITUDE_THRESHOLD	30
#define TIMER_TOLERANCE			10
#define SLOPE_TOLERANCE		 	3
#define NOT_INITIALIZED			-1
#define TIMER_TIMEOUT				1000
#define MIN_FREQUENCY				60.00
#define MAX_FREQUENCY				1500.00
#define BOTTOMPOINT  				0
#define MIDPOINT    				127
#define TOPPOINT    				255

bool ADCisSyncing(void);
uint8_t ADCread();
    
class AudioFrequencyMeter {
  public:

    AudioFrequencyMeter(void) {};
    void begin(uint32_t ulPin, uint32_t sampleRate);
    void end(void);
    
    void setClippingPin(int pin);
    void checkClipping(void);
    
    void setAmplitudeThreshold(uint8_t threshold);
    void setTimerTolerance(int tolerance);
    void setSlopeTolerance(int8_t tolerance);
    void setBandwidth(float minFrequency, float maxFrequency);
    
    float getFrequency(void);
    
  private:
  	void initializeVariables(void);
		void ADCconfigure();
		void ADCenable(void);
		void ADCdisable(void);
    void ADCsetMux(uint32_t ulPin);
    
    void tcConfigure(uint32_t sampleRate);
    bool tcIsSyncing(void);
    void tcEnable(void);
    void tcDisable(void);
    void tcReset(void);
};

