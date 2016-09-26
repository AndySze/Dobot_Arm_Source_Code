/*
  Dobot controller communication handling
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
#ifndef COMM_H
#define COMM_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define AHEAD       1
#define BACK        2
#define LEFT        3
#define RIGHT       4
#define UP          5
#define DOWN        6
#define CATCH       7
#define RELEASE     8

#define SerGetPin   48

typedef struct
{
    float x;
    float y;
    float z;
    float rHead;
    float baseAngle;
    float longArmAngle;
    float shortArmAngle;
    float pawAngle;
    float isGrab;
    float gripperAngle;
} CurrentCoordinateType;

class CommClass
{
public:
    void init(void);
    void process(void);
    void processComm0(void);
    void processComm1(void);
    void getCode(void);
    void upload(void);
    void uploadComm0(void);
    void uploadComm1(void);
    byte getSelectedCommPort(byte _SerGetPin);
public:
    float tempState, tempAxis;
    float tempX, tempY, tempZ, tempR1Head, tempIsGrab;
    float tempStartVel, tempEndVel, tempMaxVel;
    volatile unsigned int usedBufferCount;
    CurrentCoordinateType currentCoordinate;
    int voiceNum;
    int voiceState;
    bool voiceFlag;
private:
    char stateMachine;
    char numTestJ;
    float Parameter_Buf[6][10];
    byte selectedCommPort;
};

#endif
