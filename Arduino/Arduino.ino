/*
  Dobot controller project file
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

#include "Comm.h"
#include "Dobot.h"
#include "SCA61T.h"
#include "MPU6500.h"

enum {
    SensorMPU6500,
    SensorSCA61T
};

/*
 * Static variables definition
 */
static Dobot gDobotCtrl;
static CommClass gCommCtrl;
static SCA61T gSCA61T;
static MPU6500 gMPU6500;
static int gSensorType = SensorMPU6500;

/*
 * Static function declaration
 */
static void GetInfo(void);
static void GetState(void);
static void VoiceHandle(void);

/*********************************************************************************************************
** Function name:       setup
** Descriptions:        Arduino platform standard initialization
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void setup(void)
{
    /*
     * Serial connected with PC, Serial1 connected with bluetooth
     */
    Serial.begin(9600);
    Serial1.begin(9600);

    gDobotCtrl.finishFlag = true;
    // Angles
    gDobotCtrl.coordAxisH = 90 - 45;
    gDobotCtrl.coordAxisV = 45;
    // write acceleration
    gDobotCtrl.accWrite = 100;

    pinMode(40, OUTPUT);
    digitalWrite(40, LOW);
    pinMode(49, OUTPUT);
    digitalWrite(49, HIGH);

    // Init the sensor
    if( 0 == gMPU6500.init() ) {
        gSensorType = SensorMPU6500;
    }else{
        gSCA61T.init();
        gSensorType = SensorSCA61T;
    }

    if(SensorMPU6500 == gSensorType){
        // Always abandon the first 100ms' reading
        for(int i = 0; i < 5; i++) {
          gDobotCtrl.coordAxisH = gMPU6500.getRearArmAngle();
          gDobotCtrl.coordAxisV = gMPU6500.getFrontArmAngle();
          delay(20);
        }
        if(0 == gMPU6500.initState) {
          gDobotCtrl.coordAxisH = 90 - gMPU6500.getRearArmAngle();
          gDobotCtrl.coordAxisV = - gMPU6500.getFrontArmAngle();
          gDobotCtrl.coordAxisB = 0;

          digitalWrite(49, LOW);
          gDobotCtrl.stepper.initGPIO();
          gDobotCtrl.stepper.init();

          gCommCtrl.init();
        }
    }else{
        // Always abandon the first 100ms' reading
        for(int i = 0; i < 5; i++) {
          gDobotCtrl.coordAxisH = gSCA61T.getRearArmAngle();
          gDobotCtrl.coordAxisV = gSCA61T.getFrontArmAngle();
          delay(20);
        }
        if(0 == gSCA61T.initState) {
          gDobotCtrl.coordAxisH = 90 - gSCA61T.getRearArmAngle();
          gDobotCtrl.coordAxisV = gSCA61T.getFrontArmAngle();
          gDobotCtrl.coordAxisB = 0;

          digitalWrite(49, LOW);
          gDobotCtrl.stepper.initGPIO();
          gDobotCtrl.stepper.init();

          gCommCtrl.init();
        }
    }
}

/*********************************************************************************************************
** Function name:       SPI_STC_vect
** Descriptions:        SPI interrupt service routine
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
ISR (SPI_STC_vect)
{
    byte SPI_data = SPDR;

    gDobotCtrl.stepper.interDeal(SPI_data, gDobotCtrl.step1, gDobotCtrl.step2, gDobotCtrl.step3);
}

/*********************************************************************************************************
** Function name:       loop
** Descriptions:        Arduino platform standard loop subroutine
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void loop(void)
{
    static int lastState = 0;

    if (SensorMPU6500 == gSensorType) {
        if (1 == gMPU6500.initState) {
            gMPU6500.waitOffsetCmd();
            return;
        }
    } else {
        if (1 == gSCA61T.initState) {
            gSCA61T.waitOffsetCmd();
            return;
        }
    }
    gCommCtrl.process();
    if (!(gDobotCtrl.stepper.flashEnd == 1 && gDobotCtrl.stepper.numLoop == 0)) {
        return;
    }
    if(gDobotCtrl.finishFlag) {
        gDobotCtrl.step1 = 0;
        gDobotCtrl.step2 = 0;
        gDobotCtrl.step3 = 0;
        gDobotCtrl.planFlag = false;
    }
    GetInfo();
    GetState();
    switch ((int)gCommCtrl.tempState) {
        case 1:
            if(lastState != 1) {
                gDobotCtrl.length = 0;
            }
            gDobotCtrl.mouseControl(gCommCtrl.tempX, gCommCtrl.tempY, gCommCtrl.tempZ, gCommCtrl.tempR1Head, gCommCtrl.tempIsGrab);
        break;
        case 2:
            gDobotCtrl.singleVelRatio = gCommCtrl.tempStartVel / 100.0;
            gDobotCtrl.singleAxisControl(gCommCtrl.tempAxis);
        break;
        case 3:
            gDobotCtrl.visionControl(gCommCtrl.tempX, gCommCtrl.tempY, gCommCtrl.tempZ, gCommCtrl.tempR1Head, gCommCtrl.tempIsGrab, gCommCtrl.tempEndVel, gCommCtrl.tempMaxVel, gCommCtrl.tempStartVel);
        break;
        case 4:
            gDobotCtrl.write(gCommCtrl.tempStartVel, gCommCtrl.tempEndVel, gCommCtrl.tempMaxVel, gCommCtrl.tempX, gCommCtrl.tempY, gCommCtrl.tempZ, gCommCtrl.tempR1Head, gCommCtrl.tempIsGrab, gCommCtrl.tempAxis);
        break;
        case 5:
            VoiceHandle();
        break;
        case 6:
            gDobotCtrl.playback(gCommCtrl.tempX, gCommCtrl.tempY, gCommCtrl.tempZ, gCommCtrl.tempR1Head, gCommCtrl.tempIsGrab, gCommCtrl.tempEndVel, gCommCtrl.tempMaxVel, gCommCtrl.tempStartVel);
        break;
        case 7:
            gDobotCtrl.singleVelRatio = gCommCtrl.tempStartVel / 100.0;
            gDobotCtrl.singleXYZControl(gCommCtrl.tempAxis);
        break;
        case 8:
            if(lastState != 8) {
                gDobotCtrl.length = 0;
            }
            gDobotCtrl.motionControl(gCommCtrl.tempX, gCommCtrl.tempY, gCommCtrl.tempZ, gCommCtrl.tempR1Head, gCommCtrl.tempIsGrab);
        break;
        case 9:
            switch ((int)gCommCtrl.tempAxis) {
                case 0:
                    gDobotCtrl.singleVelMaxB = gCommCtrl.tempX;
                    gDobotCtrl.singleVelMaxH = gCommCtrl.tempX;
                    gDobotCtrl.singleVelMaxV = gCommCtrl.tempX;

                    gDobotCtrl.singleAccB = gCommCtrl.tempY;
                    gDobotCtrl.singleAccH = gCommCtrl.tempY;
                    gDobotCtrl.singleAccV = gCommCtrl.tempY;

                    gDobotCtrl.singleVelMaxP = gCommCtrl.tempZ;
                    gDobotCtrl.singleAccP = gCommCtrl.tempR1Head;

                    gDobotCtrl.singleVelMaxX = gCommCtrl.tempIsGrab;
                    gDobotCtrl.singleVelMaxY = gCommCtrl.tempIsGrab;
                    gDobotCtrl.singleVelMaxZ = gCommCtrl.tempIsGrab;

                    gDobotCtrl.singleAccX = gCommCtrl.tempStartVel;
                    gDobotCtrl.singleAccY = gCommCtrl.tempStartVel;
                    gDobotCtrl.singleAccZ = gCommCtrl.tempStartVel;
                break;
                case 1:
                    gDobotCtrl.stepSpdB = gCommCtrl.tempX;
                    gDobotCtrl.stepSpdH = gCommCtrl.tempX;
                    gDobotCtrl.stepSpdV = gCommCtrl.tempX;

                    gDobotCtrl.stepAccB = gCommCtrl.tempY;
                    gDobotCtrl.stepAccH = gCommCtrl.tempY;
                    gDobotCtrl.stepAccV = gCommCtrl.tempY;

                    gDobotCtrl.stepSpdP = gCommCtrl.tempZ;
                    gDobotCtrl.stepAccP = gCommCtrl.tempR1Head;

                    gDobotCtrl.stepSpdL = gCommCtrl.tempIsGrab;
                    gDobotCtrl.stepAccL = gCommCtrl.tempStartVel;

                    gDobotCtrl.height = gCommCtrl.tempMaxVel;
                break;
                case 2:
                    gDobotCtrl.accWrite = gCommCtrl.tempX;
                break;
                case 3:
                    gDobotCtrl.coordAxisH = 90 - gCommCtrl.tempX;
                    gDobotCtrl.coordAxisV = gCommCtrl.tempY;
                    gDobotCtrl.coordAxisB = 0;
                break;
                case 4:
                    if(gCommCtrl.tempX == 0) {
                        gDobotCtrl.linkBlockH = 55.07;
                        gDobotCtrl.heightMin = -30;
                    } else if(gCommCtrl.tempX == 1) {
                        gDobotCtrl.linkBlockH = 55.07;
                        gDobotCtrl.heightMin = 20;
                    } else if(gCommCtrl.tempX == 2) {
                        gDobotCtrl.linkBlockH = 49.57;
                        gDobotCtrl.heightMin = -30;
                    }
                break;
            }
            gDobotCtrl.finishFlag = true;
            gCommCtrl.tempState = 0;
        break;
        case 10:
            gDobotCtrl.velRatio = gCommCtrl.tempX / 100.0;
            gDobotCtrl.accRatio = gCommCtrl.tempY / 100.0;
            gDobotCtrl.singleVelRatio = gCommCtrl.tempZ / 100;
            gDobotCtrl.finishFlag = true;
            gCommCtrl.tempState = 0;
        break;
        case 15:
            gDobotCtrl.easyControl(gCommCtrl.tempX, gCommCtrl.tempY, gCommCtrl.tempZ, 0);
        break;
        default:

        break;
    }
    gDobotCtrl.getCurPositionXYZR(gDobotCtrl.coordAxisB, gDobotCtrl.coordAxisH, gDobotCtrl.coordAxisV, gDobotCtrl.coordAxisP);
    gCommCtrl.upload();
    gDobotCtrl.stepper.numLoop = 1;
    gDobotCtrl.stepper.flashEnd = 0;

    lastState = gCommCtrl.tempState;
}

/*********************************************************************************************************
** Function name:       GetState
** Descriptions:        Get new communication code
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
static void GetState(void)
{
    if(gDobotCtrl.finishFlag && gCommCtrl.usedBufferCount > 0) {
        gCommCtrl.getCode();
        gDobotCtrl.finishFlag = false;
    }
}

/*********************************************************************************************************
** Function name:       GetInfo
** Descriptions:        Refresh the Dobot coordinate related information
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
static void GetInfo(void)
{
    gDobotCtrl.getCurrentCoord();
    gCommCtrl.currentCoordinate.x = gDobotCtrl.coorGet.x;
    gCommCtrl.currentCoordinate.y = gDobotCtrl.coorGet.y;
    gCommCtrl.currentCoordinate.z = gDobotCtrl.coorGet.z;
    gCommCtrl.currentCoordinate.rHead = gDobotCtrl.coorGet.rHead;
    gCommCtrl.currentCoordinate.baseAngle = gDobotCtrl.coorGet.baseAngle;
    gCommCtrl.currentCoordinate.longArmAngle = gDobotCtrl.coorGet.longArmAngle;
    gCommCtrl.currentCoordinate.shortArmAngle = gDobotCtrl.coorGet.shortArmAngle;
    gCommCtrl.currentCoordinate.pawAngle = gDobotCtrl.coorGet.pawAngle;
    gCommCtrl.currentCoordinate.gripperAngle = gDobotCtrl.coorGet.gripperAngle;
}

/*********************************************************************************************************
** Function name:       VoiceHandle
** Descriptions:        Voice control
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
static void VoiceHandle()
{
    switch (gCommCtrl.voiceState) {
        case AHEAD:
            gDobotCtrl.voiceControl(gCommCtrl.voiceNum,0,0,0);
        break;
        case BACK:
            gDobotCtrl.voiceControl(-gCommCtrl.voiceNum,0,0,0);
        break;
        case LEFT:
            gDobotCtrl.voiceControl(0,gCommCtrl.voiceNum,0,0);
        break;
        case RIGHT:
            gDobotCtrl.voiceControl(0,-gCommCtrl.voiceNum,0,0);
        break;
        case UP:
            gDobotCtrl.voiceControl(0,0,gCommCtrl.voiceNum,0);
        break;
        case DOWN:
            gDobotCtrl.voiceControl(0,0,-gCommCtrl.voiceNum,0);
        break;
        case CATCH:
            gDobotCtrl.stepper.gripperCatch(45);
            gDobotCtrl.finishFlag = true;
        break;
        case RELEASE:
            gDobotCtrl.stepper.gripperRelease(0);
            gDobotCtrl.finishFlag = true;
        break;
    }
}
