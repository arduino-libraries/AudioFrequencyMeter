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

int8_t __slopeTolerance = SLOPE_TOLERANCE;
int __timerTolerance = TIMER_TOLERANCE;
uint8_t __amplitudeThreshold = AMPLITUDE_THRESHOLD;

bool __clipping;
int __clippingPin;

uint32_t __samplePin;                               // Pin used to sample the signal

uint32_t __sampleRate;                              // ADC sample rate

int  __newData, __prevData;                         // Variables to store ADC result

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
int __newMaxAmplitude;                              // Variable used to check if __maxAmplitude must be updated

int __checkMaxAmp;                                  // Used to update the new frequency in base of the AMplitude threshold

float __minFrequency;                               // Variable to store the minimum frequency that can be applied in input
float __maxFrequency;                               // Variable to store the maximum frequency that can be applied in input

void AudioFrequencyMeter::begin(uint32_t ulPin, uint32_t sampleRate)
{
#ifdef DEBUG
  pinMode(11, OUTPUT);
#endif
  __samplePin = ulPin;                              // Store ADC channel to sample
  __sampleRate = sampleRate;                        // Store sample rate value
  analogRead(A0);                                   // To start setting-up the ADC
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
  __clippingPin = pin;                              // Store the clipping pin value
  pinMode(__clippingPin, OUTPUT);
}

void AudioFrequencyMeter::checkClipping()
{
  if (__clipping) {
    digitalWrite(__clippingPin, LOW);
    __clipping = false;
  }
}

void AudioFrequencyMeter::setAmplitudeThreshold(uint8_t threshold)
{
  __amplitudeThreshold = abs(MIDPOINT - threshold);
}

void AudioFrequencyMeter::setTimerTolerance(int tolerance)
{
  __timerTolerance = tolerance;
}

void AudioFrequencyMeter::setSlopeTolerance(int8_t tolerance)
{
  __slopeTolerance = tolerance;
}

void AudioFrequencyMeter::setBandwidth(float minFrequency, float maxFrequency)
{
  __minFrequency = minFrequency;
  __maxFrequency = maxFrequency;
}

float AudioFrequencyMeter::getFrequency()
{
  if (__checkMaxAmp > __amplitudeThreshold) {
    __frequency = (float)(__sampleRate / __period);
    
    if ((__frequency < __minFrequency) || (__frequency > __maxFrequency))
      return -1;
    else
      return __frequency;
  }
  else
    return -1;
}

/*
   Private Utility Functions
*/

void AudioFrequencyMeter::initializeVariables()
{
  __clipping = false;
  __clippingPin = NOT_INITIALIZED;
  __newData = 0;
  __prevData = MIDPOINT;
  __time = 0;
  __arrayIndex = 0;
  __maxSlope = 0;
  __noMatch = 0;
  __amplitudeTimer = 0;
  __maxAmplitude = 0;
  __checkMaxAmp = 0;
  __minFrequency = MIN_FREQUENCY;
  __maxFrequency = MAX_FREQUENCY;
}

void AudioFrequencyMeter::ADCconfigure()
{
  ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
  while (ADCisSyncing())
    ;

  ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV4_Val;     // Divide Clock by 4 -> ~200kHz
  while (ADCisSyncing())
    ;


  while (ADCisSyncing())
    ;

  ADC->SAMPCTRL.reg = 0x1F;                           				// Set max Sampling Time Length
  while (ADCisSyncing())
    ;

  ADCsetMux(__samplePin);
}

bool ADCisSyncing()
{
  return (ADC->STATUS.bit.SYNCBUSY);
}

void AudioFrequencyMeter::ADCdisable()
{
  ADC->CTRLA.bit.ENABLE = 0x00;                       				// Disable ADC
  while (ADCisSyncing())
    ;
}

void AudioFrequencyMeter::ADCenable()
{
  ADC->CTRLA.bit.ENABLE = 0x01;                       				// Enable ADC
  while (ADCisSyncing())
    ;
}

void AudioFrequencyMeter::ADCsetMux(uint32_t ulPin)
{
  if ( ulPin < A0 )
  {
    ulPin += A0;
  }

  pinPeripheral(ulPin, g_APinDescription[ulPin].ulPinType);

  while (ADCisSyncing())
    ;
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ulPin].ulADCChannelNumber; // Selection for the positive ADC input
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
  uint8_t returnValue;

  while (ADCisSyncing())
    ;

  ADC->SWTRIG.bit.START = 1;

  while ( ADC->INTFLAG.bit.RESRDY == 0 );   					// Waiting for conversion to complete
  returnValue = ADC->RESULT.reg;            					// Store the value

  while (ADCisSyncing())
    ;

  ADC->SWTRIG.bit.START = 0;

  return returnValue;
}

#ifdef __cplusplus
extern "C" {
#endif

void TC5_Handler (void)
{
  __prevData = __newData;
  __newData = ADCread();

  if ((__prevData < MIDPOINT) && (__newData >= MIDPOINT)) {

    __newSlope = __newData - __prevData;

    if (abs(__newSlope - __maxSlope) < __slopeTolerance) {
      __slope[__arrayIndex] = __newSlope;
      __timer[__arrayIndex] = __time;
      __time = 0;
      
      if (__arrayIndex == 0) {
        __noMatch = 0;
        __arrayIndex++;
      }
      else if ((abs(__timer[0] - __timer[__arrayIndex]) < __timerTolerance) && (abs(__slope[0] - __newSlope) < __slopeTolerance)) {
        __totalTimer = 0;
        for (uint8_t i = 0; i < __arrayIndex; i++) {
          __totalTimer += __timer[i];
        }
        __period = __totalTimer;

        __timer[0] = __timer[__arrayIndex];
        __slope[0] = __slope[__arrayIndex];
        __arrayIndex = 1;
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

  if (__clippingPin > 0)
  {
    if (__newData == BOTTOMPOINT || __newData == TOPPOINT) {
      digitalWrite(__clippingPin, HIGH);
      __clipping = true;
    }
  }

  __time++;                             // Incremented at sampleRate
  __amplitudeTimer++;                   // Incremented at sampleRate

  __newMaxAmplitude = abs(MIDPOINT - __newData);

  if (__newMaxAmplitude > __maxAmplitude) {
    __maxAmplitude = __newMaxAmplitude;
  }

  if (__amplitudeTimer >= TIMER_TIMEOUT) {
    __amplitudeTimer = 0;
    __checkMaxAmp = __maxAmplitude;
    __maxAmplitude = 0;
  }

  TC5->COUNT16.INTFLAG.bit.MC0 = 1;     // Clear interrupt
}

#ifdef __cplusplus
}
#endif
