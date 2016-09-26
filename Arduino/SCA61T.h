/*
  SCA61T driver for Dobot
  Copyright (C) 2016 Shenzhen Yuejiang Technology Co., Ltd

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef SCA61T_h
#define SCA61T_h

// Configuration

#define FILTER_ENABLE       1           // Whether to enable filter
#define SCA1000_ENABLE      1           // Whether to use SCA1000 instead of SCA61T
#define VALID_CHECK_ENABLE  1           // Check the input of arcsin

#define PER_ARC_ANGLE 57.29577951       // PER_ARC_ANGLE=180/PI

//SPI port define
#define MISO   50                       //pin 50
#define MOSI   51                       //pin 51
#define SCK    52                       //pin 52

#define AXIS1_CSB   47
#define AXIS2_CSB   45

//SCA61T command define
#define SCA61T_MEAS   0x00
#define SCA61T_RWTR   0x08
#define SCA61T_STX    0x0E
#define SCA61T_STY    0x0F
#define SCA61T_RDAX   0x10
#define SCA61T_RDAY   0x11

//SCA61T acceleration data
#define MAX_ACC_DATA  2048
#define MIN_ACC_DATA  0
#define DIGIT_OFFSET  1024.0            //(MAX_ACC_DATA+MIN_ACC_DATA)/2
#if SCA1000_ENABLE == 0
#define SENS_DATA     819.0             //(MAX_ACC_DATA-MIN_ACC_DATA)/2
#else
#define SENS_DATA     491.0
#endif
#define ADDRESS       3072
#define SET_OFFSET_PIN 44

class SCA61T
{
public:
    SCA61T();
    void init(void);
    float getRearArmAngle(void);
    float getFrontArmAngle(void);
    void waitOffsetCmd(void);
public:
    int initState;
private:
    void setOffset(int index);
    float getTemperature(void);
    uint16_t getAverageAccelerationData(void);
    uint16_t getAverageTemperatureData(void);
    float digitalToAngle(uint16_t digitalValue);
    float digitalToTemperature(uint16_t digitalValue);

    uint16_t readAcceleration(byte command);
    byte readTemperature(byte command);
    void writeData(byte command, byte value);
    void setAxis(int iAxisID);
    float getAngle(void);
    void begin(void);
    void setup(void);
private:
    uint16_t CSB;
    int keyPressCnt;
    int deltaOffset1;
    int deltaOffset2;
};

#endif
