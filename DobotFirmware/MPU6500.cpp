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
#include <SPI.h>
#include <EEPROM.h>
#include "MPU6500.h"

/*********************************************************************************************************
** Function name:       MPU6500
** Descriptions:        Normal constructor
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
MPU6500::MPU6500()
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
void MPU6500::begin(void)
{
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(SS, OUTPUT);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE3);
    SPI.setClockDivider(SPI_CLOCK_DIV8);// different with SCA61T, speed is limited using Open-Drain output
    SPCR |= _BV(MSTR);
    SPCR |= _BV(SPE);//SPI Enable
}

/*********************************************************************************************************
** Function name:       setup
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void MPU6500::setup(void)
{
    pinMode(SET_OFFSET_PIN, INPUT);

    if(0 == digitalRead(SET_OFFSET_PIN)) {
        delay(100);
        if(0 == digitalRead(SET_OFFSET_PIN)) {
            initState =1;
            while(0 == digitalRead(SET_OFFSET_PIN)) {
            }
        }
    } else {
      initState =0;
    }
}

/*********************************************************************************************************
** Function name:       init
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
int MPU6500::init(void)
{
    begin();

    // initalize the chip select pin:
    setAxis(1);
    pinMode(CSB, OUTPUT);
    digitalWrite(CSB, HIGH);
    setAxis(2);
    pinMode(CSB, OUTPUT);
    digitalWrite(CSB, HIGH);

    if( 0x00 == checkSensorType() ){
        return 1; // 1 for error, not match
    }

    // disable I2C interface, clear register 6A[4] I2C_IF_DIS
    delay(200);
    setAxis(1);
    writeData( 0x6A, 0x10 );
    setAxis(2);
    writeData( 0x6A, 0x10 );

    setup();

    keyPressCnt = 0;

    if(0 == initState) {
        ((char *)(&deltaOffset1))[0] = EEPROM.read(ADDRESS);
        ((char *)(&deltaOffset1))[1] = EEPROM.read(ADDRESS+1);
        ((char *)(&deltaOffset1))[2] = EEPROM.read(ADDRESS+2);
        ((char *)(&deltaOffset1))[3] = EEPROM.read(ADDRESS+3);
        if( isnan( deltaOffset1 ) || abs( deltaOffset1 ) > 5.0 ){ // for case that MPU6500 not calibrated, no need for SCA1000 that FF means -1
            deltaOffset1 = 0.0;
        }
        //Serial.println(deltaOffset1);
        ((char *)(&deltaOffset2))[0] = EEPROM.read(ADDRESS+4);
        ((char *)(&deltaOffset2))[1] = EEPROM.read(ADDRESS+5);
        ((char *)(&deltaOffset2))[2] = EEPROM.read(ADDRESS+6);
        ((char *)(&deltaOffset2))[3] = EEPROM.read(ADDRESS+7);
        if( isnan( deltaOffset2 ) || abs( deltaOffset2 ) > 5.0 ){
            deltaOffset2 = 0.0;
        }
        //Serial.println(deltaOffset2);
   } else {
       Serial.println("Start calibrating...");
   }

   return 0; // for success
}

/*********************************************************************************************************
** Function name:       waitOffsetCmd
** Descriptions:        Wait for set offset command
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void  MPU6500::waitOffsetCmd(void)
{
    if (0 != digitalRead(SET_OFFSET_PIN)) {
        return;
    }
    delay(100);
    if (0 != digitalRead(SET_OFFSET_PIN)) {
        return;
    }
    keyPressCnt++;
    if(1 == keyPressCnt % 2) {
        setOffset(1);

        ((char *)(&deltaOffset1))[0] = EEPROM.read(ADDRESS);
        ((char *)(&deltaOffset1))[1] = EEPROM.read(ADDRESS+1);
        ((char *)(&deltaOffset1))[2] = EEPROM.read(ADDRESS+2);
        ((char *)(&deltaOffset1))[3] = EEPROM.read(ADDRESS+3);

        float angle = getRearArmAngle();
        Serial.print("Long arm angle new:");
        Serial.println(angle);
        if (angle > 0.5 ||
            angle < -0.5) {
            Serial.println("Long arm need to be re-calibrated!!!!!!!!!!!!");
        }
    } else {
        setOffset(2);

        ((char *)(&deltaOffset2))[0] = EEPROM.read(ADDRESS+4);
        ((char *)(&deltaOffset2))[1] = EEPROM.read(ADDRESS+5);
        ((char *)(&deltaOffset2))[2] = EEPROM.read(ADDRESS+6);
        ((char *)(&deltaOffset2))[3] = EEPROM.read(ADDRESS+7);

        float angle = getFrontArmAngle();
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
void MPU6500::setOffset(int index)
{
    const float toleratedAngle = 2;

    if(index > 2 || index <= 0) {
        return;
    }

    if(1 == index) {
        Serial.println("+++++++++++++++Calibrating long arm angle...");
        setAxis(1);
        float angle = getAngle();
        Serial.print("Long arm angle:");
        Serial.println(angle);
        if (angle > -toleratedAngle && angle < toleratedAngle) {
            Serial.println("Calibrate long arm angle successfully!");
            EEPROM.write(ADDRESS, ((char *)(&angle))[0] );
            EEPROM.write(ADDRESS+1, ((char *)(&angle))[1] );
            EEPROM.write(ADDRESS+2, ((char *)(&angle))[2] );
            EEPROM.write(ADDRESS+3, ((char *)(&angle))[3] );
            Serial.print("deltaAngle:");
            Serial.println(angle);
        } else {
            Serial.println("Calibrate long arm angle failed!!!!!!!!!!!!!!!!!!!!!!!!");
        }
    } else if (2 == index) {
        Serial.println("***************Calibrating short arm angle...");
        setAxis(2);
        float angle = getAngle();
        Serial.print("Short arm angle:");
        Serial.println(angle);
        if (angle > -toleratedAngle && angle < toleratedAngle) {
            Serial.println("Calibrate short arm angle successfully!");
            EEPROM.write(ADDRESS+4, ((char *)(&angle))[0] );
            EEPROM.write(ADDRESS+5, ((char *)(&angle))[1] );
            EEPROM.write(ADDRESS+6, ((char *)(&angle))[2] );
            EEPROM.write(ADDRESS+7, ((char *)(&angle))[3] );
            Serial.print("deltaAngle:");
            Serial.println(angle);
        } else {
            Serial.println("Calibrate short arm angle failed!!!!!!!!!!!!!!!!!!!!!!!!");
        }
    }
}

/*********************************************************************************************************
** Function name:       checkSensorType
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
int MPU6500::checkSensorType(void)
{
    byte result = 0;
    byte retVallue = 0;

    setAxis(1);
    digitalWrite(CSB, LOW);
    digitalWrite(CSB, HIGH);
    digitalWrite(CSB, LOW);

    SPI.transfer(0x80 | MPU6500_RA_WHO_AM_I);
    result = SPI.transfer(0x00);
    digitalWrite(CSB, HIGH);
    if( 0x70 == result ){
        retVallue |= 0x02;
    }

    setAxis(2);
    digitalWrite(CSB, LOW);
    digitalWrite(CSB, HIGH);
    digitalWrite(CSB, LOW);

    SPI.transfer(0x80 | MPU6500_RA_WHO_AM_I);
    result = SPI.transfer(0x00);
    digitalWrite(CSB, HIGH);
    if( 0x70 == result ){
        retVallue |= 0x01;
    }

    return retVallue;
}

/*********************************************************************************************************
** Function name:       getRearArmAngle
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      Rear arm angle
*********************************************************************************************************/
float MPU6500::getRearArmAngle(void)
{
    setAxis(1);

    return (float)getAngle() - deltaOffset1;
}

/*********************************************************************************************************
** Function name:       getFrontArmAngle
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      Front arm angle
*********************************************************************************************************/
float MPU6500::getFrontArmAngle(void)
{
    setAxis(2);

    return (float)getAngle() - deltaOffset2;
}

/*********************************************************************************************************
** Function name:       setAxis
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void MPU6500::setAxis(int iAxisID)
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
float MPU6500::getAngle(void)
{
    //Get raw angle data
#if FILTER_ENABLE == 0
    uint16_t AccData_X = readAcceleration( 0x80 | MPU6500_RA_ACCEL_XOUT_H );
    uint16_t AccData_Z = readAcceleration( 0x80 | MPU6500_RA_ACCEL_ZOUT_H );
    float AccAngle = digitalToAngle(AccData_X, AccData_Z);

    return AccAngle;
#else
    float data[BL];

    for (int i = 0; i < BL; i++) {
        uint16_t AccData_X = readAcceleration( 0x80 | MPU6500_RA_ACCEL_XOUT_H );
        uint16_t AccData_Z = readAcceleration( 0x80 | MPU6500_RA_ACCEL_ZOUT_H );
        data[i] = digitalToAngle(AccData_X, AccData_Z);
    }
    float angle = 0;
    for (int i = 0; i < BL; i++) {
        angle += data[i] * B[i];
    }
    return angle;
#endif
}

/*********************************************************************************************************
** Function name:       digitalToAngle
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      Angle value
*********************************************************************************************************/
float MPU6500::digitalToAngle(uint16_t digitalValue_X, uint16_t digitalValue_Z)
{
    //Convert data of SPI to angle
    float angleValue;
    double ax_1g, az_1g;

    // need modify for offset
    ax_1g = (1.0 * int( digitalValue_X + 0 ) ) / 16384;
    az_1g = (1.0 * int( digitalValue_Z + 0 ) ) / 16384;

    angleValue = 180.0 * atan2(-ax_1g, az_1g) / PI;

    return angleValue;
}

/*********************************************************************************************************
** Function name:       readAcceleration
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      Acceleration data
*********************************************************************************************************/
uint16_t MPU6500::readAcceleration(byte command)
{
    uint16_t result = 0;

    digitalWrite(CSB, LOW);
    digitalWrite(CSB, HIGH);
    digitalWrite(CSB, LOW);

    SPI.transfer(command);
    result = SPI.transfer(0x00) << 8;
    result |= SPI.transfer(0x00);
    digitalWrite(CSB, HIGH);

    return(result);
}

/*********************************************************************************************************
** Function name:       writeData
** Descriptions:        write data to MPU6500
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void MPU6500::writeData(byte command, byte value)
{
    digitalWrite(CSB, LOW);
    digitalWrite(CSB, HIGH);
    digitalWrite(CSB, LOW);

    SPI.transfer(command);
    SPI.transfer(value);
    digitalWrite(CSB, HIGH);
}
