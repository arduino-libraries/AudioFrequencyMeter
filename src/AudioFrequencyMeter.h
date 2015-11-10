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

#pragma once

#define ARRAY_DEPTH 				20
#define AMPLITUDE_THRESHOLD	30
#define TIMER_TOLLERANCE		10
#define SLOPE_TOLLERANCE		 3
#define NOT_INITIALIZED			-1
#define TIMEOUT							1000

//#define DEBUG								1

#include "Arduino.h"

bool ADCisSyncing(void);
uint8_t ADCread();
    
class AudioFrequencyMeter {
  public:

    AudioFrequencyMeter(void) {};
    void begin(uint32_t sampleRate);
    void end(void);
    void setClippingPin(int pin);
    void checkClipping(void);
    float getFrequency(void);
    
  private:
  	void initializeVariables(void);
		void ADCconfigure(void);
    void ADCenable(void);
    void ADCdisable(void);
    
    void tcConfigure(uint32_t sampleRate);
    bool tcIsSyncing(void);
    void tcEnable(void);
    void tcDisable(void);
    void tcReset(void);
};
