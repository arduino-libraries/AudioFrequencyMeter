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

#include "AudioFrequencyMeter.h"

#define BOTTOMPOINT  0
#define MIDPOINT    127
#define TOPPOINT    255

const int8_t __slopeTollerance = SLOPE_TOLLERANCE;
const int __timerTollerance = TIMER_TOLLERANCE;
const int __amplitudeThreshold = AMPLITUDE_THRESHOLD;

bool clipping;
int clippingPin;

uint32_t __sampleRate;                              // ADC sample rate

uint8_t  __newData, __prevData;                     // Variables to store ADC result

unsigned int __time, __totalTimer;                  // Variables used to compute period
unsigned int __period;

uint8_t __arrayIndex;                               // Index to save data in the correct position of the arrays
int __timer[ARRAY_DEPTH];                           // Array to store trigger events
int __slope[ARRAY_DEPTH];                           // Array to store changing in slope events

float __frequency;                                  // Variable to store frequency result

int __maxSlope;                                     // Variable to store max detected amplitude
int __newSlope;                                     // Variable to store a new slope

int8_t __noMatch;                                   // Variable to store non-matching trigger events

unsigned int __amplitudeTimer;                      // Variable to reset trigger
int __maxAmplitude;                                 // Variable to store the max detected amplitude

int __checkMaxAmp;

void AudioFrequencyMeter::begin(uint32_t sampleRate)
{
#ifdef DEBUG
 	pinMode(11, OUTPUT);
#endif
  __sampleRate = sampleRate;
  analogRead(A0);
  ADCdisable();
  ADCconfigure();
  ADCenable();
  tcConfigure(sampleRate);
  tcEnable();
}

void AudioFrequencyMeter::end()
{
  ADCdisable();
  tcDisable();
  tcReset();
}

void AudioFrequencyMeter::setClippingPin(int pin)
{
  clippingPin = pin;
  pinMode(clippingPin, OUTPUT);
}

void AudioFrequencyMeter::checkClipping()
{
  if (clipping) {
    digitalWrite(clippingPin, LOW);
    clipping = false;
  }
}

float AudioFrequencyMeter::getFrequency()
{
  if (__checkMaxAmp > __amplitudeThreshold) {
    __frequency = (float)(((float) __sampleRate) / ((float) __period));
  }
  return __frequency;
}

/*
   Private Utility Functions
*/

void AudioFrequencyMeter::initializeVariables()
{
  clipping = false;
  clippingPin = NOT_INITIALIZED;
  __newData = 0;
  __prevData = 0;
  __time = 0;
  __arrayIndex = 0;
  __maxSlope = 0;
  __noMatch = 0;
  __amplitudeTimer = 0;
  __maxAmplitude = 0;
  __checkMaxAmp;
}

void AudioFrequencyMeter::ADCconfigure()
{
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 |         // Divide Clock by 8.
                   ADC_CTRLB_RESSEL_8BIT;            	// 8 bits resolution

  while (ADCisSyncing())
    ;
  ADC->SAMPCTRL.reg = 0x1F;                           // Set max Sampling Time Length
  while (ADCisSyncing())
    ;
}

bool ADCisSyncing()
{
  return (ADC->STATUS.bit.SYNCBUSY);
}

void AudioFrequencyMeter::ADCdisable()
{
  ADC->CTRLA.bit.ENABLE = 0x00;                   		// Disable ADC
  while (ADCisSyncing())
    ;
}

void AudioFrequencyMeter::ADCenable()
{
  ADC->CTRLA.bit.ENABLE = 0x01;                   		// Enable ADC
  while (ADCisSyncing())
    ;
}

void AudioFrequencyMeter::tcConfigure(uint32_t sampleRate)
{
  // Enable GCLK for TCC2 and TC5 (__timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset();

  // Set Timer counter Mode to 16 bits
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

  // Set TC5 mode as match __frequency
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;

  TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
  while (tcIsSyncing())
    ;

  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0x00);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (tcIsSyncing())
    ;
}

bool AudioFrequencyMeter::tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void AudioFrequencyMeter::tcEnable()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (tcIsSyncing())
    ;
}

void AudioFrequencyMeter::tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing())
    ;
  while (TC5->COUNT16.CTRLA.bit.SWRST)
    ;
}

void AudioFrequencyMeter::tcDisable()
{
  // Disable TC5
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing())
    ;
}


uint8_t ADCread()
{
  uint32_t returnValue;
  digitalWrite(12, HIGH);
  while (ADCisSyncing())
    ;

  ADC->SWTRIG.bit.START = 1;

  while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Waiting for conversion to complete
  returnValue = ADC->RESULT.reg;            // Store the value

  while (ADCisSyncing())
    ;

  ADC->SWTRIG.bit.START = 0;
  digitalWrite(12, LOW);

  return returnValue;
}

#ifdef __cplusplus
extern "C" {
#endif

void TC5_Handler (void)
{
#ifdef DEBUG
  digitalWrite(11, LOW);
#endif
  __prevData = __newData;
  __newData = ADCread();

  if ((__prevData < MIDPOINT) && (__newData >= MIDPOINT)) {

    __newSlope = __newData - __prevData;

    if (abs(__newSlope - __maxSlope) < __slopeTollerance) {
      __slope[__arrayIndex] = __newSlope;
      __timer[__arrayIndex] = __time;
      __time = 0;
      if (__arrayIndex == 0) {
#ifdef DEBUG
 				digitalWrite(11, HIGH);
#endif
        __noMatch = 0;
        __arrayIndex++;
      }
      else if ((abs(__timer[0] - __timer[__arrayIndex]) < __timerTollerance) && (abs(__slope[0] - __newSlope) < __slopeTollerance)) { //if __timer duration and __slopes match
        __totalTimer = 0;
        for (uint8_t i = 0; i < __arrayIndex; i++) {
          __totalTimer += __timer[i];
        }
        __period = __totalTimer;
        
        __timer[0] = __timer[__arrayIndex];
        __slope[0] = __slope[__arrayIndex];
        __arrayIndex = 1;
#ifdef DEBUG
 				digitalWrite(11, HIGH);
#endif
        __noMatch = 0;
      }
      else {
        __arrayIndex++;
        if (__arrayIndex > ARRAY_DEPTH - 1) {
          __arrayIndex = 0;
          __noMatch = 0;
          __maxSlope = 0;
        }
      }
    }
    else if (__newSlope > __maxSlope) {
      __maxSlope = __newSlope;
      __time = 0;
      __noMatch = 0;
      __arrayIndex = 0;
    }
    else {
      __noMatch++;
      if (__noMatch > ARRAY_DEPTH - 1) {
        __arrayIndex = 0;
        __noMatch = 0;
        __maxSlope = 0;
      }
    }
  }


  if (__newData == BOTTOMPOINT || __newData == TOPPOINT) { //if clipping
    if (clippingPin > 0) {
      digitalWrite(clippingPin, HIGH);
      clipping = true;
    }
  }

  __time++;															// Incremented at sampleRate
  __amplitudeTimer++;										// Incremented at sampleRate

  if (abs(MIDPOINT - __newData) > __maxAmplitude) {
    __maxAmplitude = abs(MIDPOINT - __newData);
  }
  if (__amplitudeTimer >= TIMEOUT) {
    __amplitudeTimer = 0;
    __checkMaxAmp = __maxAmplitude;
    __maxAmplitude = 0;
  }

  TC5->COUNT16.INTFLAG.bit.MC0 = 1;		// Clear interrupt
}

#ifdef __cplusplus
}
#endif
