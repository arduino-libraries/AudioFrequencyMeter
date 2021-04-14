/*
  Simple Frequency Meter for Arduino Zero

  Demonstrates how to sample an input signal and get back its frequency

  This example code is in the public domain

  https://www.arduino.cc/en/Tutorial/SimpleAudioFrequencyMeter

  created by Arturo Guadalupi <a.guadalupi@arduino.cc>
  10 Nov 2015
*/

#include <AudioFrequencyMeter.h>

AudioFrequencyMeter meter;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("started");

  meter.setBandwidth(70.00, 1500);    // Ignore frequency out of this range
  meter.begin(A0, 45000);             // Initialize A0 at sample rate of 45 kHz
}

void loop() {
  // put your main code here, to run repeatedly:
  float frequency = meter.getFrequency();
  if (frequency > 0)
  {
    Serial.print(frequency);
    Serial.println(" Hz");
  }
}
