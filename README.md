# SDP3x-Arduino
Arduino library for for the SDP31 and SDP32 pressure sensors designed and manufactured by Sensirion.

## Table of Contents

1. [Install](#install)
2. [Examples](#examples)
3. [API](#api)
	1. [Public](#public)
	2. [Private](#private)
4. [License](#license)


## Install

### Option 1: Copy-Paste

Just drop `SDP3x.h` and `SDP3x.cpp` into your own project, and off you go.

### Option 2: Add ZIP as Library

1. Download: [SDP3X.zip]()
2. From the Arduino IDE:
 
  * `Sketch > Include Library > Add .ZIP Library...`
  * Navigate to `SDP3X.zip`
  * Click "OK"


## Examples

### Initialization

``` C++
#include <SDP3x.h>

using namespace SDP3X;

SDP3x sensor(Address1,MassFlow);

void setup() {
  Wire.begin();
  sensor.begin();
}
```

### Continuous Mode

``` C++
#include <SDP3x.h>

using namespace SDP3X;

SDP3x sensor(Address1,MassFlow);

void setup() {
  Wire.begin();
  sensor.begin();
  sensor.startContinuous(false);
}

bool status;
int16_t pressure;

void loop() {
  delay(1000);
  status = sensor.readMeasurement(&pressure, NULL, NULL);
}
```

## API

### Public

#### Constants

While there are many public constants in this library, there are two sets that you actually need to worry about.

**Addresses**

`Address1`, `Address2`, and `Address3` are used to specify the three possible I2C addresses for the SDP3X sensors. These are actually `0x21`, `0x22`, and `0x23` respectively, but have been made constants for ease of use.

**Temperature Compensation**

The SDP3X sensors support two different temperature compensation modes that you may choose according to your target application.

1. `MassFlow` sets the compensation mode that is optimal for applications in which you are measuring the Mass Flow Rate of a substance.
2. `DiffPressure` sets the compensation mode that is optimal for applications is which you are measuring Gauge pressure (accounts for atmospheric pressure).

#### Constructor: SDP3x(const uint8_t addr, TempCompensation comp)

This function creates a new SDP3x object and takes in the following attributes:

| Parameter | Description                                        |
| --------- | -------------------------------------------------- |
| addr      | the I2C address of the sensor (eg. Address1, 0x23) |
| comp      | the Temperature Compensation Mode (ie. MassFlow)   |

#### bool begin()

This function must be called for each sensor. It communicates with the sensor to determine the model (SDP31 or SDP32) in order to provide the correct scaling factors.

| Returns | Description                                   |
| ------- | --------------------------------------------- |
| true    | iff the initialization completed successfully |

#### bool startContinuous(bool averaging)

This function begins the continuous sampling mode of the SDP3X sensors. A new reading will be taken at 1ms intervals. When continuous sampling is occurring, each sample may either be averaged with all samples since the last time the Master read a sample (average=true) or every new sample may replace the previous value (average=false).


| Parameter | Description                                     |
| --------- | ----------------------------------------------- |
| averaging | if set to true, enable onboard sample averaging |

| Returns | Description                       |
| ------- | --------------------------------- |
| true    | iff continuous sampling has begun |

#### bool stopContinuous()

This function stops the continuous sampling process. This may be done to preserve power or because readings are no longer needed.

| Returns | Description                        |
| ------- | ---------------------------------- |
| true    | iff continuous sampling has ceased |

#### bool triggerMeasurement(bool stretching)

This function tells a sensor to take a new reading while not in continuous sampling mode. This is useful when using a timer to set the sampling rate. Triggered measurements also support clock stretching (stretching=true), which delays a Read response to the Master when a new sample is not yet available (ie. blocking read). If disabled (stretching=false), a Read will fail entirely if it occurs less than 45ms after a trigger was sent.


| Parameter  | Description             |
| ---------- | ----------------------- |
| stretching | enable clock stretching |

| Returns | Description                           |
| ------- | ------------------------------------- |
| true    | iff the trigger was sent successfully |

#### bool readMeasurement(int16_t *pressure, int16_t *temp, int16_t *scale)

This function reads the current sensor measurements (pressure and temperature). This may be used periodically (continuous mode) or in a call-back when monitoring interrupts (trigger mode). Both "temp" and "scale" should be left NULL if not used. This may reduce read times by not requesting more data than needed.

`pressure` and `temp` are returned as raw values without scale factors applied. These scale factors may be read using `getPressureScale` and `getTemperatureScale` respectively. In cases where both the SDP31 and SDP32 are used for the same application, and I2C communication is not heavy, the `scale` parameter of this function may be used rather than repeated calls to `getPressureScale` for each sensor.

| Parameter | Description                                                        |
| --------- | ------------------------------------------------------------------ |
| pressure  | a pointer to store the raw pressure value                          |
| temp      | a pointer to store the raw temperature value, NULL if not needed   |
| scale     | a pointer to store the pressure scaling factor, NULL if not needed |

| Returns | Description                             |
| ------- | --------------------------------------- |
| true    | iff the data was retrieved successfully |

#### bool readProductID(uint32_t *pid, uint64_t *serial)

This function reads the sensor's internal information. If a serial number is not needed, "serial" should be set to NULL. This will reduce read times.

| Parameter | Description                                                                  |
| --------- | ---------------------------------------------------------------------------- |
| pid       | a pointer to store the 32-bit product ID                                     |
| serial    | a pointer to store the 64-bit manufacturer serial number, NULL if not needed |

| Returns | Description                            |
| ------- | -------------------------------------- |
| true    | iff the information was read correctly |

#### bool reset()

This function resets the device to default settings.

***WARNING:*** This will reset all other I2C devices on the bus that follow this convention.

| Returns | Description                         |
| ------- | ----------------------------------- |
| true    | iff the reset was sent successfully |

#### uint8_t getPressureScale()

This function gets the Pressure Scaling Factor for this sensor. It does require communicating on the I2C bus and does not change during execution.

| Returns | Description                              |
| ------- | ---------------------------------------- |
| 0 - 255 | pressure scaling factor in units of 1/Pa |

#### uint8_t getTemperatureScale()

This function gets the Temperature Scaling Factor for this sensor. It does require communicating on the I2C bus and does not change during execution.

| Returns | Description                                |
| ------- | ------------------------------------------ |
| 0 - 255 | temperature scaling factor in units of 1/C |

### Private (Explanation only)

#### bool writeCommand(const uint8_t cmd[2])

This function writes an I2C command to the sensor.

| Parameter | Description                                  |
| --------- | -------------------------------------------- |
| cmd       | the command to write, Index 0 is the address |

| Returns | Description                          |
| ------- | ------------------------------------ |
| true    | iff the write completed successfully |

#### bool readData(uint8_t words)

This function reads multiple words of data from the the sensor. Each word consists of a 16-bit value, followed by an 8-bit CRC. Verification of the CRC is performed automatically by this function. The CRC byte only applies to the current word and is reset in between words.

| Parameter | Description                                 |
| --------- | ------------------------------------------- |
| words     | the number of words to read from the sensor |

| Returns | Description                                               |
| ------- | --------------------------------------------------------- |
| true    | iff the read completed successfully and the CRC's matched |

## License
MIT License

Copyright (c) 2018 Bryan T. Meyers

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
