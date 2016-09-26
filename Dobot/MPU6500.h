/*
  MPU6500 driver for Dobot
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
#ifndef MPU6500_H
#define MPU6500_H

// Configuration

#define FILTER_ENABLE       1           // Whether to enable filter
#define VALID_CHECK_ENABLE  1           // Check the input of arcsin

#define PER_ARC_ANGLE 57.29577951       // PER_ARC_ANGLE=180/PI

//SPI port define
#define MISO   50                       //pin 50
#define MOSI   51                       //pin 51
#define SCK    52                       //pin 52

#define AXIS1_CSB   47
#define AXIS2_CSB   45

// MPU6500 Register Address define
// Note: 0x80|Address when reading
#define MPU6500_RA_ACCEL_XOUT_H     0x3B
#define MPU6500_RA_ACCEL_XOUT_L     0x3C
#define MPU6500_RA_ACCEL_YOUT_H     0x3D
#define MPU6500_RA_ACCEL_YOUT_L     0x3E
#define MPU6500_RA_ACCEL_ZOUT_H     0x3F
#define MPU6500_RA_ACCEL_ZOUT_L     0x40
#define MPU6500_RA_TEMP_OUT_H       0x41
#define MPU6500_RA_TEMP_OUT_L       0x42
#define MPU6500_RA_WHO_AM_I         0x75

//SCA61T acceleration data
#undef ADDRESS
#define ADDRESS       3072+16
#define SET_OFFSET_PIN 44

class MPU6500
{
public:
    MPU6500();
    int init(void);
    float getRearArmAngle(void);
    float getFrontArmAngle(void);
    void waitOffsetCmd(void);
public:
    int initState;
private:
    int checkSensorType(void); // Check whether the sensor is MPU6500
    void setOffset(int index);
    float digitalToAngle(uint16_t digitalValue_X, uint16_t digitalValue_Z);

    uint16_t readAcceleration(byte command);
    void writeData(byte command, byte value);
    void setAxis(int iAxisID);
    float getAngle(void);
    void begin(void);
    void setup(void);
private:
    uint16_t CSB;
    int keyPressCnt;
    float deltaOffset1;
    float deltaOffset2;
};

#endif
