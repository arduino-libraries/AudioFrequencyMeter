# Audio Frequency Meter Library

## Functions

### `begin()`

#### Description
Initializes the TC5 timer to sample the chosen pin at the chosen frequency. begin() needs to be called before any other AudioFrequencyMeter library methods.

#### Syntax

```
AudioFrequencyMeter.begin(unsigned int pin, unsigned int sampleRate)

```

#### Parameters
pin: the analog pin to be sampled sampleRate: the frequency expressed in Hz at which the pin must be sampled

### `end()`

#### Description
Stops the timer and frees the pin chosen for the sampling.

#### Syntax

```
AudioFrequencyMeter.end()

```

#### Parameters
none

### `setClippingPin()`

#### Description
Sets a pin as output that can be used to signal a clipping event

#### Syntax

```
AudioFrequencyMeter.setClippingPin(unsigned int pin)

```

#### Parameters
pin: the pin to be used

### `checkClipping()`

#### Description
Puts the pin chosen as clipping pin HIGH in case of clipping LOW otherwise

#### Syntax

```
AudioFrequencyMeter.CheckClipping()

```

#### Parameters
none

### `setAmplitudeThreshold()`

#### Description
Sets the threshold for which a detected frequency is considered right or wrong. Default is 30

#### Syntax

```
AudioFrequencyMeter.setAmplitudeThreshold(int threshold)

```

#### Parameters
threshold: the value to be set as threshold

### `setTimerTolerance()`

#### Description
Sets the tolerance for which a sampled signal is considered valid. Default is 10

#### Syntax

```
AudioFrequencyMeter.setTimerTolerance(int tolerance)

```

#### Parameters
tolerance: the value to be set as tolerance

### `setSlopeTolerance()`

#### Description
Sets the tolerance for which the slope is valid for the trigger process. Default is 3

#### Syntax

```
AudioFrequencyMeter.setSlopeTolerance(int tolerance)

```

#### Parameters
tolerance: the value to be set as tolerance

### `setBandwidth()`

#### Description
Set the range of frequencies for which the detected frequency is valid. Default values for now are 60Hz - 1500Hz. This will be improved in future releases

#### Syntax

```
AudioFrequencyMeter.setBandwidth(float minFrequency, float maxFrequency)

```

#### Parameters
minFrequency: the lower limit to be set
maxFrequency: the upper limit to be set

### `getFrequency()`

#### Description
Returns the value of the detected frequency if it is above the threshold defined by setAmplitudeThreshold else -1

#### Syntax

```
AudioFrequencyMeter.getFrequency()
```

#### Return

The value of the detected frequency in Hz