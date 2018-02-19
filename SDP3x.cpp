/*
    SDP3x.cpp - Library for the SDP31 and SDP32 digital pressure sensors produced by Sensirion.
    Created by Bryan T. Meyers, February 14, 2017.

    Copyright (c) 2018 Bryan T. Meyers

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software
    and associated documentation files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use, copy, modify, merge, publish,
    distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all copies or
    substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
    BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "SDP3x.h"

using namespace SDP3X;

/*  Send a write command

    @param cmd - the two byte command to send
    @returns true iff all ACKs received
*/
bool SDP3x::writeCommand(const uint8_t cmd[2]) {
    size_t written;
    uint8_t status;
    Wire.beginTransmission(this->addr);
    written = Wire.write(cmd, 2);
    status  = Wire.endTransmission();
    return (status == 0) && (written == 2);
}

/*  Read data back from the device

    @param words - the number of words to read
    @returns true iff all words read and CRC passed
*/
bool SDP3x::readData(uint8_t words) {
    size_t read;
    uint8_t crc = 0xFF;
    uint8_t *next;
    bool success = true;
    // Clear buffer
    for (next = &this->buffer[12]; next > this->buffer; next--) {
        *next = 0;
    }
    // Each word is two bytes plus a CRC byte, ergo 3 bytes per word
    read = Wire.requestFrom(this->addr, (uint8_t)(words * 3));

    /*  We should have read the requested number of bytes.
        If not, we need to clear the bytes read anyways.
    */
    if (read != 3 * words) {
        success = false;
    }
    /*  Calculate CRC while reading bytes

        Adapted from:
        http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
    */
    next = this->buffer;
    for (; read > 0; read--) {
        // Read next available byte
        *next = Wire.read();
        // Every third byte
        if ((read % 3) == 1) {
            // Check CRC byte
            success = success && (crc == *next);
            crc     = 0xFF;
        } else {
            // Update CRC
            crc = CRC_LUT[crc ^ *next];
            // Go to next byte
            next++;
        }
    }
    return success;
}

/*  Constructor

    @param addr - the Address value for I2C
    @param comp - the Temperature Compensation Mode (Mass Flow or Differential Pressure)
    @returns a new SDP3X as configured
*/
SDP3x::SDP3x(const uint8_t addr, TempCompensation comp) {
    this->addr = addr;
    this->comp = comp;
}

/*  Finish Initializing the sensor object

    @returns true, iff everything went correctly
*/
bool SDP3x::begin() {
    uint32_t modelNumber;
    if (!readProductID(&modelNumber, NULL)) {
        return false;
    }
    switch (modelNumber) {
    case SDP31_PID:
        this->number = SDP31;
        return true;
    case SDP32_PID:
        this->number = SDP32;
        return true;
    default:
        /* do nothing for now */
        return false;
    }
}

/*  Begin taking continuous readings

    @param averaging - average samples until read occurs, otherwise read last value only
    @returns true, iff everything went correctly
*/
bool SDP3x::startContinuous(bool averaging) {
    switch (this->comp) {
    case MassFlow:
        if (averaging) {
            return writeCommand(StartContMassFlowAvg);
        } else {
            return writeCommand(StartContMassFlow);
        }
    case DiffPressure:
        if (averaging) {
            return writeCommand(StartContDiffPressureAvg);
        } else {
            return writeCommand(StartContDiffPressure);
        }
    default:
        return false;
    }
}

/*  Disable continuous measurements

    This may be useful to conserve power when sampling all the time is no longer necessary.
    @returns true, iff everything went correctly
*/
bool SDP3x::stopContinuous() {
    return writeCommand(StopCont);
}

/*  Start a one-shot reading

    @param averaging - average samples until read occurs, otherwise read last value only
    @returns true, iff everything went correctly
*/
bool SDP3x::triggerMeasurement(bool stretching) {
    switch (this->comp) {
    case MassFlow:
        if (stretching) {
            return writeCommand(TrigMassFlowStretch);
        } else {
            return writeCommand(TrigMassFlow);
        }
    case DiffPressure:
        if (stretching) {
            return writeCommand(TrigDiffPressureStretch);
        } else {
            return writeCommand(TrigDiffPressure);
        }
    default:
        return false;
    }
}

/*  Get a pending reading.

    This may be used periodically or in a call-back when monitoring interrupts.

    Both "temp" and "scale" should be left NULL if not used. This will reduce read times.
    @param pressure - a pointer to store the raw pressue value
    @param temp  - a pointer to store the raw temperature value
    @param scale - a pointer to store the pressure scaling factor
    @returns the raw pressure reading (no scaling)
    @returns true, iff everything went correctly
*/
bool SDP3x::readMeasurement(int16_t *pressure, int16_t *temp, int16_t *scale) {
    uint8_t words = 1;
    if (scale != NULL) {
        words = 3;
    } else if (temp != NULL) {
        words = 2;
    }
    if (!readData(words)) {
        return false;
    }
    /*  Data Format:
        | Byte  |  0  |  1  |  2  |  3  |  4  |  5  |
        | Value | pressure  | temp      | scale     |
    */
    switch (words) {
    case 3:
        if (scale != NULL) {
            *scale <<= 8;
            *scale |= (int16_t)this->buffer[4];
            *scale <<= 8;
            *scale |= (int16_t)this->buffer[5];
        }
    case 2:
        if (temp != NULL) {
            *temp <<= 8;
            *temp |= (int16_t)this->buffer[2];
            *temp <<= 8;
            *temp |= (int16_t)this->buffer[3];
        }
    case 1:
        if (pressure != NULL) {
            *pressure <<= 8;
            *pressure |= (int16_t)this->buffer[0];
            *pressure <<= 8;
            *pressure |= (int16_t)this->buffer[1];
        }
    }
    return true;
}

/*  Read back the sensor's internal information

    If a serial number is not needed, "serial" should be set to NULL. This will reduce read
   times.
    @param pid     - a pointer to store the 32-bit product ID
    @param serial  - if not null, a pointer to store the 64-bit manufacturer serial number
    @returns true, iff everything went correctly
*/
bool SDP3x::readProductID(uint32_t *pid, uint64_t *serial) {
    uint8_t words = 2;
    if (serial != NULL) {
        words = 6;
    }
    // Send first command
    if (!writeCommand(ReadInfo1)) {
        return false;
    }
    // Send decond command
    if (!writeCommand(ReadInfo2)) {
        return false;
    }
    // Read back required data
    if (!readData(words)) {
        return false;
    }
    /*  Data Format:
        | Byte  | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 |
        | Value | pid           | serial                          |
    */
    switch (words) {
    case 6:
        // "Parse" serial number
        if (serial != NULL) {
            for (words = 0; words < 8; words++) {
                *serial <<= 8;
                *serial |= (uint64_t)this->buffer[words + 4];
            }
        }
    case 2:
        // "Parse" product identifer
        if (pid != NULL) {
            for (words = 0; words < 4; words++) {
                *pid <<= 8;
                *pid |= (uint32_t)this->buffer[words];
            }
        }
    }
    return true;
}

/*  Reset the device to default settings

    WARNING: This will reset all other I2C devices that support it.
    @returns true, iff everything went correctly
*/
bool SDP3x::reset() {
    size_t written = 0;
    uint8_t status;
    Wire.beginTransmission(SoftReset[0]);
    written = Wire.write(SoftReset[1]);
    status  = Wire.endTransmission();
    return (written == 1) && (status == 0);
}

/*  Get the Pressure Scale for this sensor

    @returns scale in units of 1/Pa
*/
uint8_t SDP3x::getPressureScale() {
    switch (this->number) {
    case SDP31:
        return SDP31_DiffScale;
    case SDP32:
        return SDP32_DiffScale;
    default:
        return 1;
    }
}

/*  Get the Temperature Scale for this sensor

    @returns scale in units of 1/C
*/
uint8_t SDP3x::getTemperatureScale() {
    return SDP3X_TempScale;
}
