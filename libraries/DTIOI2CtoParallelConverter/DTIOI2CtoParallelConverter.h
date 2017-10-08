/***************************************************************************
Title: PCA9539 library (DT-I/O I2C to Parallel Converter)
by: agni

This file is free software; you can redistribute it and/or modify
it under the terms of either the GNU General Public License version 2
or the GNU Lesser General Public License version 2.1, both as
published by the Free Software Foundation.
***************************************************************************/

#ifndef DTIOI2CtoParallelConverter_h
#define DTIOI2CtoParallelConverter_h

#include <Arduino.h>
#include <Wire.h>

//P0x I/O data bus
#define PIN0_0	0
#define PIN0_1	1
#define PIN0_2	2
#define PIN0_3	3
#define PIN0_4	4
#define PIN0_5	5
#define PIN0_6	6
#define PIN0_7	7

//P1x I/O data bus
#define PIN1_0	0
#define PIN1_1	1
#define PIN1_2	2
#define PIN1_3	3
#define PIN1_4	4
#define PIN1_5	5
#define PIN1_6	6
#define PIN1_7	7


//PCA9539 Command Byte
#define INPUTPORT0	0x00
#define INPUTPORT1	0x01
#define OUTPUTPORT0	0x02
#define OUTPUTPORT1	0x03
#define POLINVPORT0	0x04
#define POLINVPORT1	0x05
#define CONFIGPORT0	0x06
#define CONFIGPORT1	0x07

#define ALLOUTPUT	0x00
#define ALLINPUT	0xFF

class DTIOI2CtoParallelConverter
{
	public :
		DTIOI2CtoParallelConverter(byte SlaveAddress);

		bool twiRead(byte &registerAddress);
		bool twiWrite(byte registerAddress, byte dataWrite);

		bool pinMode0(byte pinNumber, bool state);
		bool pinMode1(byte pinNumber, bool state);
		bool portMode0(byte value);
		bool portMode1(byte value);

		bool digitalWrite0(byte pinNumber, bool state);
		bool digitalWrite1(byte pinNumber, bool state);
		bool digitalWritePort0(byte value);
		bool digitalWritePort1(byte value);
		
		bool digitalRead0(byte &pinNumber);
		bool digitalRead1(byte &pinNumber);
		bool digitalReadPort0(byte &value);
		bool digitalReadPort1(byte &value);

	private :
		int _SlaveAddress;
};

#endif
