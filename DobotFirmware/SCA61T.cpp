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
#include <SPI.h>
#include <EEPROM.h>
#include "SCA61T.h"

/*********************************************************************************************************
** Function name:       SCA61T
** Descriptions:        Normal constructor
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
SCA61T::SCA61T()
{
    CSB = AXIS1_CSB;
}

/*********************************************************************************************************
** Function name:       begin
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void SCA61T::begin(void)
{
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(SS, OUTPUT);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPCR |= _BV(MSTR);
    SPCR |= _BV(SPE);//SPI Enable
}

void SCA61T::setup(void)
{
    pinMode(SET_OFFSET_PIN, INPUT);
    if(0 == digitalRead(SET_OFFSET_PIN)) {
        delay(100);
        if(0 == digitalRead(SET_OFFSET_PIN)) {
            initState = 1;
            while(0 == digitalRead(SET_OFFSET_PIN)) {
            }
        }
    } else {
      initState = 0;
    }
}

/*********************************************************************************************************
** Function name:       init
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void SCA61T::init(void)
{
    int8_t tmpOffset;

    begin();

    // initalize the chip select pin:
    setAxis(1);
    pinMode(CSB, OUTPUT);
    digitalWrite(CSB,HIGH);

    setAxis(2);
    pinMode(CSB, OUTPUT);
    digitalWrite(CSB,HIGH);

    setup();

    keyPressCnt =0;
    if(0==initState) {
        tmpOffset=EEPROM.read(ADDRESS);
        deltaOffset1=tmpOffset;
        tmpOffset=EEPROM.read(ADDRESS+1);
        deltaOffset2 =tmpOffset;
   } else {
       Serial.println("Start calibrating...");
   }
}

/*********************************************************************************************************
** Function name:       waitOffsetCmd
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void  SCA61T::waitOffsetCmd(void)
{
    if (0 != digitalRead(SET_OFFSET_PIN)) {
        return;
    }
    delay(100);
    if (0 != digitalRead(SET_OFFSET_PIN)) {
        return;
    }
    int8_t tmp;

    keyPressCnt++;
    if(1 == keyPressCnt % 2) {
        setOffset(1);

        tmp = EEPROM.read(ADDRESS);
        deltaOffset1 = tmp;
        float angle = getAngle();
        Serial.print("Long arm angle new:");
        Serial.println(angle);
        if (angle > 0.5 ||
            angle < -0.5) {
            Serial.println("Long arm need to be re-calibrated!!!!!!!!!!!!");
        }
    } else {
        setOffset(2);

        tmp = EEPROM.read(ADDRESS+1);
        deltaOffset2 = tmp;
        float angle = getAngle();
        Serial.print("Short arm angle new:");
        Serial.println(angle);
        if (angle > 0.5 ||
            angle < -0.5) {
            Serial.println("Short arm need to be re-calibrated!!!!!!!!!!!!");
        }
    }
    while(0 == digitalRead(SET_OFFSET_PIN)) {
    }
}

/*********************************************************************************************************
** Function name:       setOffset
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void  SCA61T::setOffset(int index)
{
    int AccData;
    int delatAcc;
    float sum=0;
    const float toleratedAngle = 2;

    if(index > 2 || index <= 0) {
        return;
    }

    if(1 == index) {
        Serial.println("+++++++++++++++Calibrating long arm angle...");
        setAxis(1);
        sum=0;
        for(int i = 0; i < 100; i++) {
            AccData = getAverageAccelerationData();
            sum += AccData - DIGIT_OFFSET;
        }
        delatAcc = sum / 100.0 + 0.5;
        float angle = (float)delatAcc / SENS_DATA;
        angle = asin((float)delatAcc / SENS_DATA) * PER_ARC_ANGLE;
        Serial.print("Long arm angle:");
        Serial.println(angle);

        if (angle > -toleratedAngle && angle < toleratedAngle) {
            Serial.println("Calibrate long arm angle successfully!");
            EEPROM.write(ADDRESS, (int)delatAcc);
            Serial.print("deltaACC:");
            Serial.println((int)delatAcc);
        } else {
            Serial.println("Calibrate long arm angle failed!!!!!!!!!!!!!!!!!!!!!!!!");
        }
    } else if (2 == index) {
        Serial.println("***************Calibrating short arm angle...");
        setAxis(2);
        sum=0;
        for(int i = 0; i < 100; i++) {
            AccData = getAverageAccelerationData();
            sum += AccData- DIGIT_OFFSET;
        }
        delatAcc = sum / 100.0 + 0.5;
        float angle = (float)delatAcc / SENS_DATA;
        angle = asin((float)delatAcc / SENS_DATA) * PER_ARC_ANGLE;
        Serial.print("Short arm angle:");
        Serial.println(angle);

        if (angle > -toleratedAngle && angle < toleratedAngle) {
            Serial.println("Calibrate short arm angle successfully!");
            EEPROM.write(ADDRESS + 1, (int)delatAcc);
            Serial.print("deltaACC:");
            Serial.println((int)delatAcc);
        } else {
            Serial.println("Calibrate short arm angle failed!!!!!!!!!!!!!!!!!!!!!!!!");
        }
    }
}

/*********************************************************************************************************
** Function name:       getRearArmAngle
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      Rear arm angle
*********************************************************************************************************/
float SCA61T::getRearArmAngle(void)
{
    setAxis(1);

    return (float)getAngle();
}

/*********************************************************************************************************
** Function name:       getFrontArmAngle
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      Front arm angle
*********************************************************************************************************/
float SCA61T::getFrontArmAngle(void)
{
    setAxis(2);

    return (float)getAngle();
}

/*********************************************************************************************************
** Function name:       setAxis
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void SCA61T::setAxis(int iAxisID)
{
    CSB = (1 == iAxisID) ? AXIS1_CSB : AXIS2_CSB;
}

#if FILTER_ENABLE == 1
const int BL = 17;
const float B[17] = {
    0.05055599666314,  0.05350226474871,  0.05615631901855,    0.058477100122,
    0.06042830269067,  0.06197912763863,  0.06310492387276,  0.06378770409005,
    0.06401652231097,  0.06378770409005,  0.06310492387276,  0.06197912763863,
    0.06042830269067,    0.058477100122,  0.05615631901855,  0.05350226474871,
    0.05055599666314
};
#endif

/*********************************************************************************************************
** Function name:       getAngle
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      Angle value
*********************************************************************************************************/
float SCA61T::getAngle(void)
{
    //Get average angle data
#if FILTER_ENABLE == 0
    uint16_t AccData = getAverageAccelerationData();
    float AccAngle = digitalToAngle(AccData);

    return AccAngle;
#else
    float data[BL];

    for (int i = 0; i < BL; i++) {
        uint16_t AccData = getAverageAccelerationData();
        data[i] = digitalToAngle(AccData);
    }
    float angle = 0;
    for (int i = 0; i < BL; i++) {
        angle += data[i] * B[i];
    }
    return angle;
#endif
}

/*********************************************************************************************************
** Function name:       getTemperature
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      Temperature value
*********************************************************************************************************/
float SCA61T::getTemperature(void)
{
    //Get average temperature data
    uint16_t TempData = getAverageTemperatureData();
    float TempAngle = digitalToTemperature(TempData);

    return TempAngle;
}

/*********************************************************************************************************
** Function name:       getAverageAccelerationData
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      Average acceleration data
*********************************************************************************************************/
uint16_t SCA61T::getAverageAccelerationData(void)
{
#if FILTER_ENABLE == 0
    uint16_t AccData = 0;

    for (uint8_t i = 0; i < 10; i++) {
        AccData += readAcceleration(SCA61T_RDAX);
        delayMicroseconds(500);
    }
    return (AccData / 10);
#else
    uint16_t AccData = 0;

    for (uint8_t i = 0; i < 1; i++) {
        AccData += readAcceleration(SCA61T_RDAX);
        delayMicroseconds(500);
    }
    return AccData;
#endif
}

/*********************************************************************************************************
** Function name:       getAverageTemperatureData
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      Average temperature data
*********************************************************************************************************/
uint16_t SCA61T::getAverageTemperatureData(void)
{
    //Get average temperature data from SPI
    uint16_t TempData = 0;

    for (uint8_t i = 0; i < 10; i++) {
        TempData += readTemperature(SCA61T_RWTR);
        delayMicroseconds(250);
    }
    return (TempData / 10);
}

/*********************************************************************************************************
** Function name:       digitalToAngle
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      Angle value
*********************************************************************************************************/
float SCA61T::digitalToAngle(uint16_t digitalValue)
{
    //Convert data of SPI to angle
    float angleValue;
    float offset=1024;

    if (digitalValue>MAX_ACC_DATA) {
        digitalValue = 2 * MAX_ACC_DATA - digitalValue;
    } else if (digitalValue<MIN_ACC_DATA) {
        digitalValue = 2 * MIN_ACC_DATA - digitalValue;
    }
    if( AXIS1_CSB == CSB) {
        offset = DIGIT_OFFSET + deltaOffset1;
    } else if(AXIS2_CSB == CSB) {
         offset = DIGIT_OFFSET + deltaOffset2;
    }
#if VALID_CHECK_ENABLE == 0
    angleValue = asin((digitalValue - offset) / SENS_DATA) * PER_ARC_ANGLE;
#else
    angleValue = (digitalValue - offset) / SENS_DATA;
    if (angleValue > 1.0) {
        angleValue = 1.0;
    } else if (angleValue < -1.0) {
        angleValue = -1.0;
    }
    angleValue = asin(angleValue) * PER_ARC_ANGLE;
#endif
    return angleValue;
}

/*********************************************************************************************************
** Function name:       digitalToTemperature
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      Temperature data
*********************************************************************************************************/
float SCA61T::digitalToTemperature(uint16_t digitalValue)
{
    float temperature;
    temperature = (digitalValue - 197.0) / (-1.083);
    return temperature;
}

/*********************************************************************************************************
** Function name:       readAcceleration
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
uint16_t SCA61T::readAcceleration(byte command)
{
    uint16_t result = 0;
    digitalWrite(CSB, LOW);
    SPI.transfer(command);
    result = SPI.transfer(0x00) << 8;
    result |= SPI.transfer(0x00);
    digitalWrite(CSB, HIGH);
    return(result >> 5);
}

/*********************************************************************************************************
** Function name:       readTemperature
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
byte SCA61T::readTemperature(byte command)
{
    byte result = 0;
    digitalWrite(CSB, LOW);
    SPI.transfer(command);
    result = SPI.transfer(0x00);
    digitalWrite(CSB, HIGH);
    return(result);
}

/*********************************************************************************************************
** Function name:       writeData
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void SCA61T::writeData(byte command, byte value)
{
    digitalWrite(CSB, LOW);
    SPI.transfer(command);
    SPI.transfer(value);
    digitalWrite(CSB, HIGH);
}
