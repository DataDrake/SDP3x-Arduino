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

#ifndef SDP3X_H
#define SDP3X_H

#if defined(ARDUINO) && (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

/* Assertions are needed to prevent invalid configurations */
#include <assert.h>

/* The SDP3X namespace will be used to prevent collisions with other libraries */
namespace SDP3X {

    /* Model allows us to specify which digital SDP3x sensor is detected */
    enum Model{ SDP31, SDP32 };

    /*  Address specifies the valid addresses for SDP3x sensors.

        We pre-shift the constant to open up the LSB for R/W bit
    */
    enum Address {
        Addr21 = 0x21 << 1,
        Addr22 = 0x22 << 1,
        Addr23 = 0x23 << 1
    };

    /* AddressMasks define the interal bit masks for each address */
    const unsigned char AddressMask21 = 0x01;
    const unsigned char AddressMask22 = 0x02;
    const unsigned char AddressMask23 = 0x04;


    /* I2C Command Definitions */
    const unsigned char StartContMassFlowAvg[2]     = { 0x36, 0x03 };
    const unsigned char StartContMassFlow[2]        = { 0x36, 0x08 };
    const unsigned char StartContDiffPressureAvg[2] = { 0x36, 0x15 };
    const unsigned char StartContDiffPressure[2]    = { 0x36, 0x1E };
    const unsigned char StopContinuous[2]           = { 0x3F, 0xF9 };
    const unsigned char TrigMassFlow[2]             = { 0x36, 0x24 };
    const unsigned char TrigMassFlowStretch[2]      = { 0x37, 0x26 };
    const unsigned char TrigDiffPressure[2]         = { 0x36, 0x2F };
    const unsigned char TrigDiffPressureStretch[2]  = { 0x37, 0x2D };
    const unsigned char ReadInfo1[2]                = { 0x36, 0x7C };
    const unsigned char ReadInfo2[2]                = { 0xE1, 0x02 };
    const unsigned char SoftReset[2]                = { 0x00, 0x06 };

    /*  TempCompensation is used to set the temperature compensation mode for the sensor

        MassFlow     - Use this mode for Mass Flow applications
        DiffPressure - Use this mode for Differential Pressure applications (absolute pressure matters)
    */
    enum TempCompensation {
        MassFlow,
        DiffPressure
    }

    /*  Model-Specific Parameters

        Pressure Units    - 1/Pa
        Temperature Units - 1/C
    */
    const unsigned int SDP31_PID = 0x03010188;
    const unsigned int SDP32_PID = 0x03010288;
    const signed char SDP31_DiffScale = 60;
    const signed char SDP32_DiffScale = 240;
    const signed char SDP3X_TempScale = 200;

    /* The SDP3X class can be used to interface with both the SDP31 and SDP32 sensors */
    public class SDP3X {
        private:
            /* The sensor model number*/
            Model number;
            /* The address of this device */
            Address addr;
            /* The address mask of this device */
            unsigned char mask;
            /* Keeps track of which addresses are in use in order to avoid collisions */
            static unsigned char addressesUsed;
            /* The Temperature Compensation mode to use */
            TempCompensation comp;

            /*  Send a write command

                @param cmd - the two byte command to send
                @return true iff all ACKs received
            */
            bool WriteCommand(unsigned char cmd[2]);

            /*  Send a read command

                @return true iff ACK received
            */
            bool ReadCommand();

            /*  Read data back from the device

            */
            bool ReadData(unsigned short *data, unsigned char crc, bool stop)
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
            signed short ReadContinuous(unsigned short *temp, unsigned short *scale);

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
            signed short ReadTrigger(unsigned short *temp, unsigned short *scale);

            /*  Read back the sensor's internal information

                If a serial number is not needed, "serial" should be set to NULL. This will reduce read times.
                @param serial  - if not null, a pointer to store the 64-bit manufacturer serial number
                @returns a 32-bit product identifier
            */
            unsigned int ReadProductID(unsigned long *serial);

            /* Reset the device to default settings */
            void Reset();

            /*  Get the Pressure Scale for this sensor

                @returns scale in units of 1/Pa
            */
            signed char GetPressureScale();

            /*  Get the Temperature Scale for this sensor

                @returns scale in units of 1/C
            */
            signed char GetTemperatureScale();
    }
}

#endif
