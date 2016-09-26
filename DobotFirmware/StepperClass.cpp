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
#include "StepperClass.h"

/*********************************************************************************************************
** Function name:       initGPIO
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::initGPIO(void)
{
    spiStateMachine = 0;
    spiReceiveNum  = 0;
    servoAngle[0] = 1000;
    servoAngle[1] = 500;

    pinMode(pumpPin, OUTPUT);digitalWrite(pumpPin,LOW);
    pinMode(valvePin,OUTPUT);digitalWrite(valvePin,LOW);
    pinMode(LaserPin,OUTPUT);digitalWrite(LaserPin,LOW);
    //disable FPGA SPI
    pinMode(NSS_Pin,OUTPUT);  digitalWrite(NSS_Pin,LOW);
    pinMode(EN_Pin,OUTPUT);  digitalWrite(EN_Pin,LOW);

    numLoop = 0;
    flashEnd = 0;
}

/*********************************************************************************************************
** Function name:       init
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::init(void)
{
    SPSR = 0x00;
    SPCR |= _BV(SPIE);//SPI Interrupt Enable
    SREG |=0x80;//Global Interrupt enable

    //set SPI as slave
    pinMode(MISO, OUTPUT);
    pinMode(MOSI,INPUT);
    pinMode(SCK,INPUT);
    pinMode(SS,INPUT);
    SPCR &= ~_BV(MSTR);
    SPCR |= _BV(DORD);//LSB first
    SPCR &= ~_BV(CPOL); // resume after using MPU6500
    SPCR |= _BV(CPHA);//sample at Trailing Edge
    SPCR |= _BV(SPE);//SPI Enable

    delay(100);

    //enable FPGA SPI
    digitalWrite(NSS_Pin,LOW);
    delay(100);
    digitalWrite(EN_Pin,HIGH);
    //handRotation(0);
    gripperRelease(0);
}

/*********************************************************************************************************
** Function name:       interDeal
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::interDeal(byte _SPI_data,int _step1, int _step2, int _step3)
{
    byte SPI_data = _SPI_data;

    if(spiStateMachine == 0) {
        if(SPI_data == 0xa5) {
            // Receive header
            spiStateMachine =1;
            spiReceiveNum = 1;
        } else {
            spiReceiveNum = 0;
        }
    } else if(spiStateMachine == 1) {
        spiReceiveNum++;
        // Length OK!
        if(spiReceiveNum == 8) {
            spiStateMachine = 0;
            spiReceiveNum = 0;
            if(SPI_data == 0x5a) {
                spiStep(_step1,_step2,_step3);
                numLoop = 0; flashEnd = 1;
            }
        } else if(spiReceiveNum > 8) {
            spiStateMachine = 0;    spiReceiveNum = 0;
        }
    }
}

/*********************************************************************************************************
** Function name:       spiStep
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::spiStep(int _step1, int _step2, int _step3)
{
    step(_step1, _step2, _step3);
    digitalWrite(NSS_Pin, HIGH);
    SPCR &= ~_BV(SPIE);// Turn off SPI interrupt
    byte t = 0;

    for(spiReceiveNum = 0; spiReceiveNum < SPISENUM; spiReceiveNum++) {
        SPDR = spiSeBuffer[spiReceiveNum];
        while (!(SPSR & _BV(SPIF))) {
        }
        t = SPDR;
    }
    spiStateMachine = 0;
    spiReceiveNum = 0;
    digitalWrite(NSS_Pin, LOW);
    SPCR |= _BV(SPIE);//Turn on SPI interrupt
}

/*********************************************************************************************************
** Function name:       clcFan
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::clcFan(void)
{
    double tempFan=0;
    int axisNum;

    for(axisNum = 0 ; axisNum < AXIS_SUM; axisNum++) {
        if (pulseOut[axisNum] == 0) {
            tempFan = FREQCONST_MAX_F;
        } else {
            tempFan = CLK_PER_TERM / pulseOut[axisNum] - 1;
        }
        if (tempFan > FREQCONST_MAX_F) {
            tempFan = FREQCONST_MAX_F;
        } else if (tempFan <= FREQCONST_MIN_F) {
            tempFan = FREQCONST_MIN_F;
        } else {
        }
        freqConst[axisNum] = (unsigned long)tempFan;
        residue[axisNum] = (unsigned char)((tempFan - freqConst[axisNum]) * 256);
    }
}

/*********************************************************************************************************
** Function name:       getPackage
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::getPackage(void)
{
    for(uint32_t spiSeCount = 0; spiSeCount < SPISENUM; spiSeCount++) {
        spiSeBuffer[spiSeCount] = 0x00;
    }
    spiSeBuffer[0] = 0xa5;
    spiSeBuffer[1] = freqConst[0];
    spiSeBuffer[2] = freqConst[0] >> 8;
    spiSeBuffer[3] = freqConst[0] >> 16;
    spiSeBuffer[4] = residue[0];
    spiSeBuffer[5] = freqConst[1];
    spiSeBuffer[6] = freqConst[1] >> 8;
    spiSeBuffer[7] = freqConst[1] >> 16;
    spiSeBuffer[8] = residue[1];
    spiSeBuffer[9] = freqConst[2];
    spiSeBuffer[10] = freqConst[2] >> 8;
    spiSeBuffer[11] = freqConst[2] >> 16;
    spiSeBuffer[12] = residue[2];

    if(axisDir[0] == 1) {
        spiSeBuffer[13] = spiSeBuffer[13] | 0x01;
    } else if(axisDir[0] == -1) {
        spiSeBuffer[13] = spiSeBuffer[13] & 0xFE;
    }
    if(axisDir[1] == 1) {
        spiSeBuffer[13] = spiSeBuffer[13] | 0x02;
    } else if(axisDir[1] == -1) {
        spiSeBuffer[13] = spiSeBuffer[13] & 0xFD;
    }
    if(axisDir[2] == 1) {
        spiSeBuffer[13] = spiSeBuffer[13] | 0x04;
    } else if(axisDir[2] == -1) {
        spiSeBuffer[13] = spiSeBuffer[13] & 0xFB;
    }
    spiSeBuffer[14] = servoAngle[0] & 0xff;
    spiSeBuffer[15] = servoAngle[0] >> 8;
    spiSeBuffer[16] = servoAngle[1] & 0xff;
    spiSeBuffer[17] = servoAngle[1] >> 8;
    spiSeBuffer[SPISENUM - 1] = 0x5a;
}

/*********************************************************************************************************
** Function name:       step
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::step(int step1, int step2, int step3)
{
    pulseOut[0] = abs(step1);
    pulseOut[1] = abs(step2);
    pulseOut[2] = abs(step3);
    clcFan();
    axisDir[0] = (step1 >= 0) ? 1 : -1;
    axisDir[1] = (step2 >= 0) ? 1 : -1;
    axisDir[2] = (step3 >= 0) ? 1 : -1;
    getPackage();
}

/*********************************************************************************************************
** Function name:       handRotation
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::handRotation(float value)
{
    if (value > 90.00) {
        value = 90.00;
    } else if (value < -90.00) {
        value = -90.00;
    }
    servoAngle[1] = (unsigned int)(value * 1000.0 / 180.0 + 1500.0 - 1000);
}

/*********************************************************************************************************
** Function name:       gripperCatch
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::gripperCatch(float value)
{
    servoAngle[0] = (unsigned int)(value * 1000.0 / 180.0 + 1500.0 - 1000);
    digitalWrite(valvePin, HIGH); // valve close
    digitalWrite(pumpPin, HIGH); // pump enable
}

/*********************************************************************************************************
** Function name:       gripperRelease
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::gripperRelease(float value)
{
    servoAngle[0] = (unsigned int)(value * 1000.0 / 180.0 + 1500.0 - 1000);
    digitalWrite(valvePin, LOW); // valve open, decompression
    digitalWrite(pumpPin, LOW);   // pump disnable
}

/*********************************************************************************************************
** Function name:       gripperRotation
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::gripperRotation(float value)
{
    servoAngle[0] = (unsigned int)(value * 1000.0 / 180.0 + 1500.0 - 1000);
}

/*********************************************************************************************************
** Function name:       laserOn
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::laserOn(void)
{
    digitalWrite(LaserPin, HIGH);
}

/*********************************************************************************************************
** Function name:       laserOff
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void StepperClass::laserOff(void)
{
    digitalWrite(LaserPin,LOW);
}
