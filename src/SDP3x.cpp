/*
    SDP3x.h - Library for the SDP31 and SDP32 digital pressure sensors produced by Sensirion.
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

namespace SDP3X {

    /* The SDP3X class can be used to interface with both the SDP31 and SDP32 sensors */
    public class SDP3X {

        private:
            /* The sensor model number*/
            Model number;
            /* The address of this device */
            Address addr;
            /* The address mask of this device */
            uint8_t mask;
            /* Keeps track of which addresses are in use in order to avoid collisions */
            static uint8_t addressesUsed;
            /* The Temperature Compensation mode to use */
            TempCompensation comp;
            /* Internal buffer to reuse for reads */
            uint8_t buffer[18];

            /*  Send a write command

                @param addr - the I2C address to use
                @param cmd - the two byte command to send
                @return true iff all ACKs received
            */
            bool WriteCommand(uint8_t addr, uint8_t cmd[2]) {
                size_t written = 0;
                Wire.beginTransmission(this.mask);
                written = Wire.write(cmd, 2);
                Wire.endTransmission();
                return written == 2;
            }

            /*  Read data back from the device

                @param addr  - the I2C address to use
                @param words - the number of words to read
                @param addr - the I2C address to use
            */
            bool ReadData(uint8_t addr, uint8_t words){
                size_t read, i, j, byte;
                uint16_t crc = 0xFF00;
                uint16_t next;
                bool failed = false;
                // Each word is two bytes blus a CRC byte, ergo 3 bytes per word
                read = Wire.requestFrom(addr, 3*words);
                if read != 3*words {
                    return false;
                }
                byte = 0;
                for( i=0; i < read; i++){
                    next = (uint16_t)Wire.read();
                    // CRC
                    if( (i&3) == 2 ){
                        next << 8;
                        failed = !(crc & next == next);
                    } else {
                        crc |= next;
                        for( j=0; j < 8; j++ ){
                            crc = crc << 1;
                            if( crc & 0x8000 == 0x8000 ) {
                                crc &= 0x3100;
                            }
                        }
                        this.buffer[byte] = (uint8_t)next;
                        byte++;
                    }
                }
                return failed;
            }

        public:
            /*  Constructor

                @param addr - the Address value for I2C
                @param comp - the Temperature Compensation Mode (Mass Flow or Differential Pressure)
                @returns a new SDP3X as configured
            */
            SDP3X(Address addr, TempCompensation comp);

            /*  Begin taking continuous readings

                @param averaging - average samples until read occurs, otherwise read last value only
            */
            void StartContinuous(bool averaging);

            /*  Take a continuous reading

                Both "temp" and "scale" should be left NULL if not used. This will reduce read times.
                @param temp  - a pointer to store the raw temperature value
                @param scale - a pointer to store the pressure scaling factor
                @returns the raw pressure reading (no scaling)
            */
            int16_t ReadContinuous(int16_t *temp, int16_t *scale);

            /*  Disable continuous measurements

                This may be useful to conserve power when sampling all the time is no longer necessary.
            */
            void StopContinuous();

            /*  Start a one-shot reading

                @param averaging - average samples until read occurs, otherwise read last value only
            */
            void TriggerMeasurement(bool stretching);

            /*  Complete a triggered reading.

                This may be used periodically or in a call-back when monitoring interrupts.

                Both "temp" and "scale" should be left NULL if not used. This will reduce read times.
                @param temp  - a pointer to store the raw temperature value
                @param scale - a pointer to store the pressure scaling factor
                @returns the raw pressure reading (no scaling)
            */
            int16_t ReadTrigger(int16_t *temp, int16_t *scale);

            /*  Read back the sensor's internal information

                If a serial number is not needed, "serial" should be set to NULL. This will reduce read times.
                @param serial  - if not null, a pointer to store the 64-bit manufacturer serial number
                @returns a 32-bit product identifier
            */
            uint32_t ReadProductID(uint64_t *serial);

            /* Reset the device to default settings */
            void Reset();

            /*  Get the Pressure Scale for this sensor

                @returns scale in units of 1/Pa
            */
            uint8_t GetPressureScale();

            /*  Get the Temperature Scale for this sensor

                @returns scale in units of 1/C
            */
            uint8_t GetTemperatureScale();
    }
}

#endif
