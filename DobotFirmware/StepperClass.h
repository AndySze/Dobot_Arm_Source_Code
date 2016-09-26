/*
  Stepper driver for Dobot, for which interact with FPGA
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
#ifndef STEPPERCLASS_H
#define STEPPERCLASS_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define num_sum 654

#define AXIS_SUM            3
#define SET_POS             1
#define SET_NEG             -1
#define SET_STOP            0
#define CLK_PER_TERM        (double)500000
#define FREQCONST_MAX_F     (double)1000000
#define FREQCONST_MIN_F     (double)2
#define SPISENUM            19
#define NSS_Pin             42
#define EN_Pin              40

#define pumpPin             41
#define valvePin            43
#define LaserPin            12
#define btn_D44             44
#define btn_D46             46
#define EN_Ang1             45
#define EN_Ang2             47

#define HAND_ANGLE_OPEN     0
#define HAND_ANGLE_CLOSE    45

class StepperClass
{
public:
    void initGPIO(void);
    void init(void);
    void handRotation(float value);
    void gripperCatch(float value);
    void gripperRelease(float value);
    void gripperRotation(float value);
    void laserOn(void);
    void laserOff(void);
    void interDeal(byte _SPI_data,int _step1, int _step2, int _step3);
public:
    volatile char flashEnd, numLoop;
private:
    void step(int step1, int step2, int step3);
    void pumpOn(void);
    void pumpOff(void);
    void valveOpen(void);
    void valveClose(void);
    void clcFan(void);
    void getPackage(void);
    void spiStep(int _step1,int _step2,int _step3);
private:
    uint32_t pulseOut[AXIS_SUM];

    volatile unsigned char spiSeBuffer[SPISENUM];
    volatile uint32_t servoAngle[2];
    volatile byte spiReceiveNum;
    volatile byte spiStateMachine;

    int axisDir[AXIS_SUM];
    unsigned long freqConst[AXIS_SUM];
    byte residue[AXIS_SUM];

    boolean gripperRst;
    unsigned long delayLoop;
};

#endif
