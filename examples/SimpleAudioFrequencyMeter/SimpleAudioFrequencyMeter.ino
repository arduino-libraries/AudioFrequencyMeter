/*
  Simple Frequency Meter for Arduino Zero

  Demonstrates how to sample an input signal and get back its frequency

  This example code is in the public domain

  http://arduino.cc/en/Tutorial/SimpleAudioFrequencyMeter

  created by Arturo Guadalupi <a.guadalupi@arduino.cc>
  10 Nov 2015
*/

#include <AudioFrequencyMeter.h>

AudioFrequencyMeter meter;

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
  meter.begin(45000);     // Intialize sample rate at 45kHz
}

void loop() {
  // put your main code here, to run repeatedly:
  float frequency = meter.getFrequency();
 meter.getFrequency();
 SerialUSB.print(frequency);
 SerialUSB.println(" Hz");
}
