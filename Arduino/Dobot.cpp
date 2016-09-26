/*
  Dobot controller API
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
#include "Dobot.h"

/*********************************************************************************************************
** Function name:       Dobot
** Descriptions:        Normal constructor
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
Dobot::Dobot(void)
{
    stepLastB = 0;
    stepLastH = 0;
    stepLastV = 0;
    stepLastP = 0;
    stepLastL = 0;

    stepSpdB = 200;
    stepSpdH = 200;
    stepSpdV = 200;
    stepSpdP = 200;

    stepSpdL = 100;
    stepAccL = 200;

    stepAccB = 200;
    stepAccH = 200;
    stepAccV = 200;
    stepAccP = 200;

    stepDir1 = -1;
    stepDir2 = 1;
    stepDir3 = -1;

    periodCount = 0;
    periodTime = 0.02;

    finishFlag = false;
    planFlag = false;

    stepAngle = 1.8;
    subdiv1 = 16;
    subdiv2 = 16;
    subdiv3 = 16;
    gearRatio1 = 10;
    gearRatio2 = 10;
    gearRatio3 = 10;

    speedB = 0;
    speedH = 0;
    speedV = 0;
    speedP = 0;

    remainPulseB = 0;
    remainPulseH = 0;
    remainPulseV = 0;
    remainPulseP = 0;

    step1 = 0;
    step2 = 0;
    step3 = 0;
    step4 = 0;

    sendPulseSumB = 0;
    sendPulseSumH = 0;
    sendPulseSumV = 0;
    sendPulseSumP = 0;

    sendSpeedBSum = 0;
    sendSpeedHSum = 0;
    sendSpeedVSum = 0;
    sendSpeedPSum = 0;

    arm1Len = 135;
    arm2Len = 160;
    baseV = 0;
    baseH = 0;
    linkBlockV = 0;
    linkBlockH = 55.07;

    coordAxisB = 0;
    coordAxisH = 45;
    coordAxisV = 45;
    coordAxisP = 0;

    coordX = 0;
    coordY = 0;
    coordZ = 0;
    coordR = 0;

    singleVel = 0;
    singleVelMax = 20;
    singleAcc = 20;

    singleVelMaxB = 15;
    singleVelMaxH = 15;
    singleVelMaxV = 15;
    singleVelMaxP = 30;

    singleAccB = 50;
    singleAccH = 50;
    singleAccV = 50;
    singleAccP = 50;

    singleVelMaxX = 60;
    singleVelMaxY = 60;
    singleVelMaxZ = 60;
    singleVelMaxR = 60;

    singleAccX = 60;
    singleAccY = 60;
    singleAccZ = 60;
    singleAccR = 60;

    axisB.v = 50;
    axisH.v = 50;
    axisV.v = 50;
    axisP.v = 50;

    axisB.a = 200;
    axisH.a = 200;
    axisV.a = 200;
    axisP.a = 200;

    accWrite = 100;

    height = 10;
    transFlag = false;
    jumPlanFlag = false;
    downFlag = false;
    pauseFlag = false;
    pauseCount = 0;
    pauseTime = 200;

    velRatio = 0.5;
    accRatio = 0.5;

    singleVelRatio = 0.5;

    heightMin = -30;
    limitFlag = false;
    firstLimitFlag = false;
}

/*********************************************************************************************************
** Function name:       mouseControl
** Descriptions:        API for mouse control
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::mouseControl(float dx,float dy,float dz,float r,float isGrab)
{
    float h, v, b;

    if (finishFlag == false && planFlag == false) {
        samplingPeriod = 0.1;

        stepSpdB = 200;
        stepSpdH = 200;
        stepSpdV = 200;
        stepSpdP = 200;

        stepAccB = 100;
        stepAccH = 100;
        stepAccV = 100;
        stepAccP = 100;

        handPosition[0] = dx;
        handPosition[1] = dy;
        handPosition[2] = dz;

        for (int i = 0; i < length; i++) {
            dx += handPosition[0] + handHistory[i][0];
            dy += handPosition[1] + handHistory[i][1];
            dz += handPosition[2] + handHistory[i][2];
        }
        length = length+ 1;
        if (length > SMOOTHING_FRAMES) {
            for (int i = 0; i < SMOOTHING_FRAMES - 1; i++) {
                handHistory[i][0] = handHistory[i+1][0];
                handHistory[i][1] = handHistory[i+1][1];
                handHistory[i][2] = handHistory[i+1][2];
            }
            length = length - 1;
        }
        handHistory[length-1][0] = handPosition[0];
        handHistory[length-1][1] = handPosition[1];
        handHistory[length-1][2] = handPosition[2];

        dx = dx / length;
        dy = dy / length;
        dz = dz / length;

        h = sqrt(coordX * coordX + coordY * coordY) + 0.4 * dy;
        v= coordZ + 0.4 * dz;
        b = coordAxisB - dx / 400.0 * 90;

        if(v > ARM_HEIGHT_MAX) {
            v = ARM_HEIGHT_MAX;
        } else if (v < heightMin) {
            v = heightMin;
        }
        sendH = h;
        sendV = v;
        sendB = b;
    }
    movjHVBP(sendH, sendV,sendB, 0);
    if(isGrab == 1) {
        stepper.gripperCatch(45);
    } else if(isGrab == 0) {
        stepper.gripperRelease(0);
    }
    if(r >= -90 && r <= 90) {
        stepper.handRotation(r);
        coordAxisP = r;
    }
}

/*********************************************************************************************************
** Function name:       API for easy control
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::easyControl(float a1,float a2,float b,float p)
{
    double x,y,z;

    if (finishFlag == false && planFlag == false) {
        samplingPeriod = 0.1;

        stepSpdB = 200;
        stepSpdH = 200;
        stepSpdV = 200;
        stepSpdP = 200;

        stepAccB = 500;
        stepAccH = 500;
        stepAccV = 500;
        stepAccP = 500;

        if(a1 > ARM_A1_MAX) {
            a1 = ARM_A1_MAX;
        } else if (a1 < ARM_A1_MIN) {
           a1 = ARM_A1_MIN;
        }
        if(a2> ARM_A2_MAX) {
            a2 = ARM_A2_MAX;
        } else if (a2 <ARM_A2_MIN) {
            a2 = ARM_A2_MIN;
        }
        handPosition[0] = a1;
        handPosition[1] = a2;
        handPosition[2] = b;

        x = 0;
        y = 0;
        z = 0;

        length = length + 1;

        for (int i = 0; i < length; i++) {
            x += handHistory[i][0];
            y += handHistory[i][1];
            z += handHistory[i][2];
        }
        if (length > SMOOTHING_FRAMES) {
            for (int i = 0; i < SMOOTHING_FRAMES - 1; i++) {
                handHistory[i][0] = handHistory[i + 1][0];
                handHistory[i][1] = handHistory[i + 1][1];
                handHistory[i][2] = handHistory[i + 1][2];
            }
            length = length - 1;
        }
        handHistory[length - 1][0] = handPosition[0];
        handHistory[length - 1][1] = handPosition[1];
        handHistory[length - 1][2] = handPosition[2];

        line.s = fabs(a1 - coordAxisH) + fabs(a2 - coordAxisV) + fabs(b - coordAxisB);

        sendH = x / length;
        sendV = y / length;
        sendB = z / length;
    }
    movjABS(sendH, sendV,sendB,0);
}

/*********************************************************************************************************
** Function name:       singleAxisControl
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::singleAxisControl(int state)
{
    if(state == 9) {
        stepper.gripperCatch(coordAxisG);
        finishFlag = true;
    } else if (state == 10) {
        stepper.gripperRelease(coordAxisG);
        finishFlag = true;
    } else if(state == 13) {
        stepper.laserOn();
        finishFlag = true;
    } else if(state == 14) {
        stepper.laserOff();
        finishFlag = true;
    } else {
        single(state);
        stepper.handRotation(coordAxisP);
        stepper.gripperRotation(coordAxisG);
    }
}

/*********************************************************************************************************
** Function name:       singleXYZControl
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::singleXYZControl(int state)
{
    if( state == 9) {
        stepper.gripperCatch(coordAxisG);
        finishFlag = true;
    } else if (state == 10) {
        stepper.gripperRelease(coordAxisG);
        finishFlag = true;
    } else if(state == 13) {
        stepper.laserOn();
        finishFlag = true;
    } else if(state == 14) {
        stepper.laserOff();
        finishFlag = true;
    } else {
        singleXY(state);
        stepper.handRotation(coordAxisP);
        stepper.gripperRotation(coordAxisG);
    }
}

/*********************************************************************************************************
** Function name:       write
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::write(float vs, float ve, float vm, float dx, float dy, float dz,float stopflag, float isLaserOn,float state)
{
    planSpeed(vs, ve, vm, dx, dy,dz,stopflag,isLaserOn,state);
}

/*********************************************************************************************************
** Function name:       playback
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::playback(float b, float a1, float a2, float p, float isGrab, float gripperAngle, float delayTime, int type)
{
    pauseTime = delayTime;

    if (type == JMP) {
        jump(b, a1, a2, p);
    } else if (type == JOINT) {
        movj(b, a1, a2, p);
    } else if(type == LINE) {
        movl(b, a1, a2, p);
    }
    stepper.handRotation(coordAxisP);
    if(catchFlag) {
        if(isGrab == 1) {
            stepper.gripperCatch(gripperAngle);
        } else if(isGrab == 0) {
            stepper.gripperRelease(gripperAngle);
        }
        coordAxisG = gripperAngle;
    }
}

/*********************************************************************************************************
** Function name:       visionControl
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::visionControl(float x, float y, float z, float r,float isGrab,float  gripperAngle, float delayTime,int type)
{
    pauseTime = delayTime;

    if (type == JMP) {
        jumpXYZ(x, y, z, r);
    } else if (type == JOINT) {
        movjXYZ(x, y, z, r);
    } else if(type == LINE) {
        movlXYZ(x, y, z, r);
    }
    stepper.handRotation(coordAxisP);
    if(catchFlag) {
        if(isGrab == 1) {
            stepper.gripperCatch(gripperAngle);
        } else if(isGrab == 0) {
            stepper.gripperRelease(gripperAngle);
        }
        coordAxisG = gripperAngle;
    }
}

/*********************************************************************************************************
** Function name:       motionControl
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::motionControl(float x, float y, float z, float r, float isGrab)
{
    double h, v, b;

    if (finishFlag == false && planFlag ==false) {
        samplingPeriod = 0.1;

        stepSpdB = 200;
        stepSpdH = 200;
        stepSpdV = 200;
        stepSpdP = 200;

        stepAccB = 500;
        stepAccH = 500;
        stepAccV = 500;
        stepAccP = 500;

        handPosition[0] = x;
        handPosition[1] = y;
        handPosition[2] = z;

        x = 0;
        y = 0;
        y = 0;

        length = length + 1;

        for (int i = 0; i < length; i++) {
            x += handHistory[i][0];
            y += handHistory[i][1];
            z += handHistory[i][2];
        }
        if (length > SMOOTHING_FRAMES) {
            for (int i = 0; i < SMOOTHING_FRAMES - 1; i++) {
                handHistory[i][0] = handHistory[i + 1][0];
                handHistory[i][1] = handHistory[i + 1][1];
                handHistory[i][2] = handHistory[i + 1][2];
            }
            length = length - 1;
        }
        handHistory[length - 1][0] = handPosition[0];
        handHistory[length - 1][1] = handPosition[1];
        handHistory[length - 1][2] = handPosition[2];

        x = handPosition[0] * 0.5 + x / length * 0.5;
        y = handPosition[1] * 0.5 + y / length * 0.5;
        z = handPosition[2] * 0.5 + z / length * 0.5;

        h = 130 + (0.5 * x + 100);
        b = y ;
        v = 0.5 * (z - 200) + heightMin;

        r = coordR;
        line.s = sqrt((h - coordX) * (h - coordX) + (b - coordY) * (b - coordY) + (v - coordZ) * (v - coordZ));
        if( v > ARM_HEIGHT_MAX) {
            v = ARM_HEIGHT_MAX;
        } else if (v < heightMin) {
            v = heightMin;
        }

        sendH = h;
        sendV = v;
        sendB = b;
        samplingPeriod = 0.1;
        if (line.s / 0.1 > 200) {
            samplingPeriod = line.s / 200.0;
        }
    }
     movjXYZEven(sendH, sendB,sendV, 0);
    if(isGrab == 1) {
        stepper.gripperCatch(45);
    } else if(isGrab == 0) {
        stepper.gripperRelease(0);
    }
}

/*********************************************************************************************************
** Function name:       voiceControl
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot:: voiceControl(float dx, float dy, float dz, float dp)
{
    movXYZ(dx, dy, dz, dp);
}

/*********************************************************************************************************
** Function name:       movjACC
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::movjACC(float x, float y,float z,float r)
{
    if(!finishFlag) {
        if (!planFlag) {
            float h, v, b, p;
            float a1,a2;

            float l1, l2;
            float distV, distH, dist;
            float theta, theta1, theta2;

            axisB.a = stepAccB * accRatio;
            axisH.a = stepAccH * accRatio;
            axisV.a = stepAccV * accRatio;
            axisP.a = stepAccP*accRatio;

            axisB.v = stepSpdB * velRatio;
            axisH.v = stepSpdH * velRatio;
            axisV.v = stepSpdV * velRatio;
            axisP.v = stepSpdP * velRatio;

            h = sqrt(x * x + y * y);
            v = z;
            b = atan2(y, x) / PI * 180;
            p = r - b;

            sendServoP = p;
            l1 = arm1Len;
            l2 = arm2Len;
            distV = v - baseV + linkBlockV;
            distH = h - baseH - linkBlockH;
            dist = sqrt(distH * distH + distV * distV);
            theta = atan(distV / distH);
            if(fabs((l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist)) > 1) {
                finishFlag = true;
                planFlag = false;
                step1 = 0;
                step2 = 0;
                step3 = 0;
                step4 = 0;
                return;
            }
            theta1 = acos((l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist)) + theta;
            theta2 = atan2((l1 * sin(theta1) - distV), (distH - l1 * cos(theta1)));

            solvedH = 90 - theta1 / PI * 180;
            solvedV = theta2 / PI * 180;

            solvedB = b;
            solvedP = p;

            if (solvedH < ARM_A1_MIN || solvedH > ARM_A1_MAX || solvedV < ARM_A2_MIN || solvedV > ARM_A2_MAX) {
                finishFlag = true;
                planFlag = false;
                step1 = 0;
                step2 = 0;
                step3 = 0;
                step4 = 0;
                return;
            }
            a1 = solvedH;
            a2 = solvedV;

            targetB = b;
            targetH = a1;
            targetV = a2;
            targetP = p;

            stepLastB = coordAxisB;
            stepLastH = coordAxisH;
            stepLastV = coordAxisV;
            stepLastP = coordAxisP;

            axisB.s = b - stepLastB;
            dirB = 1;

            if (axisB.s < 0) {
                axisB.s = -axisB.s;
                dirB = -1;
            }
            axisB.finishFlag = axisB.s < 1e-6;
            x = axisB.v * axisB.v / axisB.a;
            if (x < axisB.s) {
                axisB.t1 = axisB.v / axisB.a;
                axisB.t2 = (axisB.s - x) / axisB.v;
                axisB.T = 2 * axisB.t1 + axisB.t2;
            } else {
                v = sqrt(axisB.a * axisB.s);
                axisB.t1 = v / axisB.a;
                axisB.t2 = 0;
                axisB.T =  2 * axisB.t1;
            }
            T = axisB.T;
            axisH.s = a1 - stepLastH;
            dirH = 1;
            if (axisH.s < 0) {
                axisH.s = - axisH.s;
                dirH = -1;
            }
            axisH.finishFlag = axisH.s < 1e-6;
            x = axisH.v * axisH.v / axisH.a;
            if (x < axisH.s) {
                axisH.t1 = axisH.v / axisH.a;
                axisH.t2 = (axisH.s - x) / axisH.v;
               axisH.T =  2 * axisH.t1 + axisH.t2;
            } else {
                v = sqrt(axisH.a * axisH.s);
                axisH.t1 = v / axisH.a;
                axisH.t2 = 0;
               axisH.T =  2 * axisH.t1;
            }
            if (axisH.T > T) {
                T = axisH.T;
            }
            axisV.s = a2 - stepLastV;
            dirV = 1;
            if (axisV.s < 0) {
                axisV.s = - axisV.s;
                dirV = -1;
            }
            axisV.finishFlag = axisV.s < 1e-6;
            x = axisV.v * axisV.v / axisV.a;

            if (x < axisV.s) {
                axisV.t1 = axisV.v / axisV.a;
                axisV.t2 = (axisV.s - x) / axisV.v;
                axisV.T = 2 * axisV.t1 + axisV.t2;
            } else {
                v = sqrt(axisV.a * axisV.s);
                axisV.t1 = v / axisV.a;
                axisV.t2 = 0;
                axisV.T = 2 * axisV.t1;
            }
            if (axisV.T > T) {
                T =  axisV.T;
            }
            axisP.s = p - stepLastP;
            dirP = 1;
            if (axisP.s < 0) {
                axisP.s = -axisP.s;
                dirP = -1;
            }
            axisP.finishFlag = axisP.s < 1e-6;
            x = axisP.v * axisP.v / axisP.a;
            if (x < axisP.s) {
                axisP.t1 = axisP.v / axisP.a;
                axisP.t2 = (axisP.s - x) / axisP.v;
               axisP.T =  2 * axisP.t1 + axisP.t2;
            } else {
                v = sqrt(axisP.a * axisP.s);
                axisP.t1 = v / axisP.a;
                axisP.t2 = 0;
               axisP.T =  2 * axisP.t1;
            }
            if (axisP.T > T) {
                T = axisP.T;
            }
            if (2 * axisB.s / T <= axisB.v) {
                axisB.t1 = T / 2;
                axisB.t2 = 0;
                axisB.a = axisB.s / axisB.t1 / axisB.t1;
            } else {
                axisB.t1 = T - axisB.s / axisB.v;
                axisB.t2 = T - 2 * axisB.t1;
                axisB.a = axisB.v / axisB.t1;
            }
            axisB.v = axisB.a * axisB.t1;
            if (2 * axisH.s / T <= axisH.v) {
                axisH.t1 = T / 2;
                axisH.t2 = 0;
                axisH.a = axisH.s / axisH.t1 / axisH.t1;
            } else {
                axisH.t1 = T - axisH.s / axisH.v;
                axisH.t2 = T - 2 * axisH.t1;
                axisH.a = axisH.v / axisH.t1;
            }
            axisH.v = axisH.a * axisH.t1;
            if (2 * axisV.s / T <= axisV.v) {
                axisV.t1 = T / 2;
                axisV.t2 = 0;
                axisV.a =  axisV.s / axisV.t1 / axisV.t1;
            } else {
                axisV.t1 = T - axisV.s / axisV.v;
                axisV.t2 = T - 2 * axisV.t1;
                axisV.a = axisV.v / axisV.t1;
            }
            axisV.v = axisV.a * axisV.t1;
            if (2 * axisP.s / T <= axisP.v) {
                axisP.t1 = T / 2;
                axisP.t2 = 0;
                axisP.a =  axisP.s / axisP.t1 / axisP.t1;
            } else {
                axisP.t1 = T - axisP.s / axisP.v;
                axisP.t2 = T - 2 * axisP.t1;
                axisP.a = axisP.v / axisP.t1;
            }
            axisP.v = axisP.a * axisP.t1;
            planFlag = true;
            periodCount = 0;
        }
        if (!(axisB.finishFlag && axisH.finishFlag && axisV.finishFlag && axisP.finishFlag)) {
            getSpeedJoint();
        } else {
            finishFlag = true;
            planFlag = false;
            periodCount = 0;
            step1 = 0;
            step2 = 0;
            step3 = 0;
            step4 = 0;
        }
    }
}

/*********************************************************************************************************
** Function name:       movjHVBP
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::movjHVBP(float h, float v,float b,float p)
{
    if(!finishFlag) {
        if (!planFlag) {
            T = samplingPeriod;
            float a1, a2;

            float l1, l2;
            float distV, distH, dist;
            float theta, theta1, theta2;

            axisB.a = stepAccB;
            axisH.a = stepAccH;
            axisV.a = stepAccV;
            axisP.a = stepAccP;

            l1 = arm1Len;
            l2 = arm2Len;
            distV = v - baseV + linkBlockV;
            distH = h - baseH - linkBlockH;
            dist = sqrt(distH * distH + distV * distV);
            theta = atan(distV / distH);
            if(fabs((l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist)) >1) {
                finishFlag = true;
                planFlag = false;
                step1 = 0;
                step2 = 0;
                step3 = 0;
                step4 = 0;
                return;
            }
            theta1 = acos((l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist)) + theta;
            theta2 = atan2((l1 * sin(theta1) - distV), (distH - l1 * cos(theta1)));

            solvedH = 90 - theta1 / PI * 180;
            solvedV = theta2 / PI * 180;

            solvedB = b;
            solvedP = p;

            if (solvedH > ARM_A1_MAX || solvedV < ARM_A2_MIN || solvedV > ARM_A2_MAX) {
                finishFlag = true;
                planFlag = false;
                step1 = 0;
                step2 = 0;
                step3 = 0;
                step4 = 0;
                return;
            }
            if(solvedH < ARM_A1_MIN) {
                solvedH = ARM_A1_MIN;
            }
            a1 = solvedH;
            a2 = solvedV;

            targetB = b;
            targetH = a1;
            targetV = a2;
            targetP = p;

            stepLastB = coordAxisB;
            stepLastH = coordAxisH;
            stepLastV = coordAxisV;

            stepLastP = p;

            axisB.s = b - stepLastB;
            axisB.finishFlag = fabs(axisB.s) < 1e-6;
            axisB.v = axisB.s / T;

            axisH.s = a1 - stepLastH;
            axisH.finishFlag = fabs(axisH.s) < 1e-6;
            axisH.v = axisH.s / T;

            axisV.s = a2 - stepLastV;
            axisV.finishFlag = fabs(axisV.s) < 1e-6;
            axisV.v = axisV.s / T;

            axisP.s = p - stepLastP;
            axisP.finishFlag = fabs(axisP.s) < 1e-6;
            axisP.v = axisP.s / T;

            planFlag = true;
            periodCount = 0;
        }
        if (!(axisB.finishFlag && axisH.finishFlag && axisV.finishFlag && axisP.finishFlag)) {
            getSpeedHVBP();
            if(axisB.finishFlag && axisH.finishFlag && axisV.finishFlag && axisP.finishFlag) {
                finishFlag = true;
                planFlag = false;
                periodCount = 0;
            }
        }
    }
}

/*********************************************************************************************************
** Function name:       movjABS
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::movjABS(float a1, float a2,float b,float p)
{
    if(!finishFlag) {
        if (!planFlag) {
            T = 0.08;

            if (a1 > ARM_A1_MAX || a2 < ARM_A2_MIN || a2 > ARM_A2_MAX) {
                finishFlag = true;
                planFlag = false;
                step1 = 0;
                step2 = 0;
                step3 = 0;
                step4 = 0;
                return;
            }
            if(a1 < ARM_A1_MIN) {
                a1 = ARM_A1_MIN;
            }
            targetB = b;
            targetH = a1;
            targetV = a2;
            targetP = p;

            stepLastB = coordAxisB;
            stepLastH = coordAxisH;
            stepLastV = coordAxisV;

            stepLastP = p;

            axisB.s = b - stepLastB;
            axisB.finishFlag = fabs(axisB.s) < 1e-6;
            axisB.v = axisB.s / T;

            axisH.s = a1 - stepLastH;
            axisH.finishFlag = fabs(axisH.s) < 1e-6;
            axisH.v = axisH.s / T;

            axisV.s = a2 - stepLastV;
            axisV.finishFlag = fabs(axisV.s) < 1e-6;
            axisV.v = axisV.s / T;

            axisP.s = p - stepLastP;
            axisP.finishFlag = fabs(axisP.s) < 1e-6;
            axisP.v = axisP.s / T;

            planFlag = true;
            periodCount = 0;
        }
        if (!(axisB.finishFlag && axisH.finishFlag && axisV.finishFlag && axisP.finishFlag)) {
            getSpeedHVBP();
            if(axisB.finishFlag && axisH.finishFlag && axisV.finishFlag && axisP.finishFlag) {
                finishFlag = true;
                planFlag = false;
                periodCount = 0;
            }
        }
    }
}

/*********************************************************************************************************
** Function name:       movjXYZEven
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::movjXYZEven(float x, float y,float z,float r)
{
    if(!finishFlag) {
        if (!planFlag) {
            float h, v, b, p;

            T = samplingPeriod;
            float a1, a2;

            float l1, l2;
            float distV, distH, dist;
            float theta, theta1, theta2;

            axisB.a = stepAccB;
            axisH.a = stepAccH;
            axisV.a = stepAccV;
            axisP.a = stepAccP;

            h = sqrt(x * x + y * y);
            v = z;
            b = atan2(y, x) / PI * 180;
            p = r - b;

            l1 = arm1Len;
            l2 = arm2Len;
            distV = v - baseV + linkBlockV;
            distH = h - baseH - linkBlockH;
            dist = sqrt(distH * distH + distV * distV);
            theta = atan(distV / distH);
            if(fabs((l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist)) > 1) {
                finishFlag = true;
                planFlag = false;
                step1 = 0;
                step2 = 0;
                step3 = 0;
                step4 = 0;
                return;
            }
            theta1 = acos((l1 * l1 + dist * dist - l2 * l2)/(2 * l1 * dist)) + theta;
            theta2 = atan2((l1 * sin(theta1) - distV), (distH - l1 * cos(theta1)));

            solvedH = 90 - theta1 / PI * 180;
            solvedV = theta2 / PI * 180;

            solvedB = b;
            solvedP = p;

            if (solvedH > ARM_A1_MAX || solvedV < ARM_A2_MIN || solvedV > ARM_A2_MAX) {
                finishFlag = true;
                planFlag = false;
                step1 = 0;
                step2 = 0;
                step3 = 0;
                step4 = 0;
                return;
            }
            if(solvedH < ARM_A1_MIN) {
                solvedH = ARM_A1_MIN;
            }
            a1 = solvedH;
            a2 = solvedV;

            targetB = b;
            targetH = a1;
            targetV = a2;
            targetP = p;

            stepLastB = coordAxisB;
            stepLastH = coordAxisH;
            stepLastV = coordAxisV;

            stepLastP = p;

            axisB.s = b - stepLastB;
            axisB.finishFlag = fabs(axisB.s) < 1e-6;
            axisB.v = axisB.s / T;

            axisH.s = a1 - stepLastH;
            axisH.finishFlag = fabs(axisH.s) < 1e-6;
            axisH.v = axisH.s / T;

            axisV.s = a2 - stepLastV;
            axisV.finishFlag = fabs(axisV.s) < 1e-6;
            axisV.v = axisV.s / T;

            axisP.s = p - stepLastP;
            axisP.finishFlag = fabs(axisP.s) < 1e-6;
            axisP.v = axisP.s / T;

            planFlag = true;
            periodCount = 0;
        }
        if (!(axisB.finishFlag && axisH.finishFlag && axisV.finishFlag && axisP.finishFlag)) {
            getSpeedHVBP();
            if(axisB.finishFlag && axisH.finishFlag && axisV.finishFlag && axisP.finishFlag) {
                finishFlag = true;
                planFlag = false;
                periodCount = 0;
            }
        }
    }
}

/*********************************************************************************************************
** Function name:       getSpeedHVBP
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::getSpeedHVBP()
{
    periodCount++;

    speedB = axisB.v;
    speedH = axisH.v;
    speedV = axisV.v;
    speedP = axisP.v;

    if (!axisB.finishFlag) {
        if (periodCount >= int(T / periodTime)) {
            axisB.finishFlag = true;
        } else {
            stepLastB += speedB * periodTime;
        }
    } else {
        speedB = 0;
    }

    if (!axisH.finishFlag) {
        if (periodCount >= int(T / periodTime)) {
            axisH.finishFlag = true;
        } else {
            stepLastH += speedH * periodTime;
        }
    } else {
        speedH = 0;
    }

    if (!axisV.finishFlag) {
        if (periodCount >= int(T / periodTime)) {
            axisV.finishFlag = true;
        } else {
            stepLastV += speedV * periodTime;
        }
    } else {
        speedV = 0;
    }

    if (!axisP.finishFlag) {
        if (periodCount >= int(T / periodTime)) {
            axisP.finishFlag = true;
        } else {
            stepLastP += speedP * periodTime;
        }
    } else {
        speedP = 0;
    }
    writePulse(speedB, speedH, speedV, speedP);
}

/*********************************************************************************************************
** Function name:       setStepSpeed
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::setStepSpeed(char stepNum, float stepSpeed)
{
    switch(stepNum) {
        case STEP_B:
            stepSpdB = stepSpeed;
            break;
        case STEP_H:
            stepSpdH = stepSpeed;
            break;
        case STEP_V:
            stepSpdV = stepSpeed;
            break;
        case STEP_P:
            stepSpdP = stepSpeed;
            break;
        default: break;
    }
}

/*********************************************************************************************************
** Function name:       setStepDir
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::setStepDir(int dir1, int dir2, int dir3)
{
    stepDir1 = dir1;
    stepDir2 = dir2;
    stepDir3 = dir3;
}

/*********************************************************************************************************
** Function name:       getCurPositionXYZR
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::getCurPositionXYZR(float b, float a1, float a2, float p)
{
    float l1, l2;
    float theta1, theta2;
    float h, v;

    l1 = arm1Len;
    l2 = arm2Len;
    theta1 = PI / 2 - a1 / 180 * PI;
    theta2 = a2 / 180 * PI;
    h = l1 * cos(theta1) + l2 * cos(theta2) + baseH + linkBlockH;
    v = l1 * sin(theta1) - l2 * sin(theta2) + baseV - linkBlockV;;
    coordX = h * cos(b / 180 * PI);
    coordY = h * sin(b / 180 * PI);
    coordZ = v;
    coordR = p + b;
}

/*********************************************************************************************************
** Function name:       getDistPositionXYZR
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::getDistPositionXYZR(float b, float a1, float a2, float p)
{
    float l1, l2;
    float theta1, theta2;
    float h, v;

    l1 = arm1Len;
    l2 = arm2Len;
    theta1 = PI / 2 - a1 / 180 * PI;
    theta2 = a2 / 180 * PI;
    h = l1 * cos(theta1) + l2 * cos(theta2) + baseH + linkBlockH;
    v = l1 * sin(theta1) - l2 * sin(theta2) + baseV - linkBlockV;;
    distX = h * cos(b / 180 * PI);
    distY = h * sin(b / 180 * PI);
    distZ = v;
    distR = p + b;
}

/*********************************************************************************************************
** Function name:       getNextPositionXYZR
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::getNextPositionXYZR(float b, float a1, float a2, float p, CurrentCoordinateType *coord)
{
    float l1, l2;
    float theta1, theta2;
    float h, v;

    l1 = arm1Len;
    l2 = arm2Len;
    theta1 = PI / 2 - a1 / 180 * PI;
    theta2 = a2 / 180 * PI;
    h = l1 * cos(theta1) + l2 * cos(theta2) + baseH + linkBlockH;
    v = l1 * sin(theta1) - l2 * sin(theta2) + baseV - linkBlockV;
    coord->x = h * cos(b / 180 * PI);
    coord->y = h * sin(b / 180 * PI);
    coord->z = v;
    coord->rHead = p + b;
}

/*********************************************************************************************************
** Function name:       jump
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::jump(float b, float a1, float a2, float p)
{
    if(!finishFlag) {
        if (!jumPlanFlag) {
            getDistPositionXYZR(b, a1, a2, p);
            orgX = coordX;
            orgY = coordY;
            orgZ = coordZ;
            orgR = coordR;
            sendServoP = p;
            jumPlanFlag = true;
            catchFlag = false;
        }
        if (!pauseFlag) {
            if (!finishFlag && !transFlag) {
                movjACC(orgX, orgY, orgZ + height, orgR);
            }
            if (finishFlag && !transFlag) {
                transFlag = true;
                finishFlag = false;
            }
            if (!finishFlag && transFlag && !downFlag) {
                movjACC(distX, distY, distZ + height, distR);
            }
            if (finishFlag && transFlag) {
                downFlag = true;
                finishFlag = false;
            }
            if (!finishFlag && downFlag) {
                movjACC(distX, distY, distZ, distR);
            }
            if(finishFlag && downFlag) {
                pauseFlag = true;
                finishFlag = false;
            }
        }
        if(pauseFlag) {
            step1 = 0;
            step2 = 0;
            step3 = 0;
            step4 = 0;
            pauseCount++;

            if(pauseCount > (pauseTime * 500.0 / 20)) {
                catchFlag = true;
            }
            if(pauseCount > (pauseTime * 1000.0 / 20)) {
                finishFlag = true;
                planFlag = false;
                pauseCount = 0;
                pauseFlag = false;
                jumPlanFlag = false;
                transFlag = false;
                downFlag = false;
            }
        }
    }
}

/*********************************************************************************************************
** Function name:       jumpXYZ
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::jumpXYZ(float x, float y, float z, float r)
{
    if(!finishFlag) {
        if (!jumPlanFlag) {
            distX = x;
            distY = y;
            distZ = z;
            distR = r;
            orgX = coordX;
            orgY = coordY;
            orgZ = coordZ;
            orgR = coordR;
            jumPlanFlag = true;
            catchFlag = false;
        }
        if (!pauseFlag) {
            if (!finishFlag && !transFlag) {
                movjACC(orgX, orgY, orgZ + height, orgR);
            }
            if (finishFlag && !transFlag) {
                transFlag = true;
                finishFlag = false;
            }
            if (!finishFlag && transFlag && !downFlag) {
                movjACC(distX, distY, distZ + height, distR);
            }
            if (finishFlag && transFlag) {
                downFlag = true;
                finishFlag = false;
                stepper.handRotation(sendServoP);
            }
            if (!finishFlag && downFlag) {
                movjACC(distX, distY, distZ, distR);
            }
            if(finishFlag && downFlag) {
                pauseFlag = true;
                finishFlag = false;
            }
        }
        if(pauseFlag) {
            step1 = 0;
            step2 = 0;
            step3 = 0;
            step4 = 0;
            pauseCount++;

            if(pauseCount > (pauseTime * 500.0 / 20)) {
                catchFlag = true;
            }
            if(pauseCount > (pauseTime * 1000.0 / 20)) {
                finishFlag = true;
                planFlag = false;
                pauseCount = 0;
                pauseFlag = false;
                jumPlanFlag = false;
                transFlag = false;
                downFlag = false;
            }
        }
    }
}

/*********************************************************************************************************
** Function name:       movXYZ
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::movXYZ(float dx, float dy, float dz, float dr)
{
    float s,v;

    if (!finishFlag) {
        if (!planFlag) {
            if (dx > 0) {
                line.s = dx;
                dirX = 1;
                dirY = 0;
                dirZ = 0;
            } else if (dx < 0) {
                line.s = -dx;
                dirX = -1;
                dirY = 0;
                dirZ = 0;
            } else if (dy > 0) {
                line.s = dy;
                dirX = 0;
                dirY = 1;
                dirZ = 0;
            } else if (dy < 0) {
                line.s = -dy;
                dirX = 0;
                dirY = -1;
                dirZ = 0;
            } else if (dz > 0) {
                line.s = dz;
                dirX = 0;
                dirY = 0;
                dirZ = 1;
            } else if (dz < 0) {
                line.s = -dz;
                dirX = 0;
                dirY = 0;
                dirZ = -1;
            }
            orgX = coordX;
            orgY = coordY;
            orgZ = coordZ;

            if (stepSpdL == 0) {
                finishFlag = true;
                planFlag = false;
                return;
            }
            line.v = stepSpdL;
            line.a = stepAccL;
            s = line.v * line.v / line.a;

            if (s < line.s) {
                line.t1 = line.v / line.a;
                line.t2 = (line.s - s) / line.v;
                line.T =  2 * line.t1 + line.t2;
            } else {
                v = sqrt(line.a * line.s);
                line.t1 = v / line.a;
                line.t2 = 0;
                line.T =  2 * line.t1;
                line.v = v;
            }
            T1 = line.t1;
            T2 = line.t2;
            T = line.T;

            line.finishFlag = false;
            planFlag = true;
            periodCount = 0;
        }
        if (!line.finishFlag) {
            getSpeedLine();
        } else {
            finishFlag = true;
            planFlag = false;
            periodCount = 0;
            stepLastL = 0;
            step1 = 0;
            step2 = 0;
            step3 = 0;
            step4 = 0;
        }
    }
}

/*********************************************************************************************************
** Function name:       movl
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::movl(float b, float a1, float a2, float p)
{
    float s,v;

    if (!finishFlag && !pauseFlag) {
        if (!planFlag) {
            sendServoP = p;
            getDistPositionXYZR(b, a1, a2, p);

            line.s = sqrt((distX - coordX) * (distX - coordX) + (distY - coordY) * (distY - coordY) + (distZ - coordZ) * (distZ - coordZ));
            dirX = (distX - coordX) / line.s;
            dirY = (distY - coordY) / line.s;
            dirZ = (distZ - coordZ) / line.s;

            orgX = coordX;
            orgY = coordY;
            orgZ = coordZ;

            line.v = stepSpdL * velRatio;
            line.a = stepAccL * accRatio;
            s = line.v*line.v / line.a;

            if (s < line.s) {
                line.t1 = line.v / line.a;
                line.t2 = (line.s - s) / line.v;
                line.T =  2 * line.t1 + line.t2;
            } else {
                v = sqrt(line.a * line.s);
                line.t1 = v / line.a;
                line.t2 = 0;
                line.T =  2 * line.t1;
                line.v = v;
            }
            T1 = line.t1;
            T2 = line.t2;
            T = line.T;

            line.finishFlag = false;
            planFlag = true;
            periodCount = 0;
            catchFlag = false;
        }
        if (!line.finishFlag) {
            getSpeedLine();
        } else {
            finishFlag = true;
            planFlag = false;
            coordAxisP = p;
            periodCount = 0;
            stepLastL = 0;
            step1 = 0;
            step2 = 0;
            step3 = 0;
            step4 = 0;
        }
    }
    if (finishFlag && !pauseFlag) {
        finishFlag = false;
        pauseFlag = true;
    }
    if(pauseFlag) {
        step1 = 0;
        step2 = 0;
        step3 = 0;
        step4 = 0;
        pauseCount++;
        if(pauseCount > (pauseTime * 500.0 / 20)) {
            catchFlag = true;
        }
        if(pauseCount > (pauseTime * 1000.0 / 20)) {
            finishFlag = true;
            pauseCount = 0;
            pauseFlag = false;
        }
    }
}

/*********************************************************************************************************
** Function name:       movlXYZ
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::movlXYZ(float x, float y, float z, float r)
{
    float s,v;

    if (!finishFlag && !pauseFlag) {
        if (!planFlag) {

            line.s = sqrt((x - coordX) * (x - coordX) + (y - coordY) * (y - coordY) + (z - coordZ) * (z - coordZ));
            dirX = (x - coordX) / line.s;
            dirY = (y - coordY) / line.s;
            dirZ = (z - coordZ) / line.s;

            orgX = coordX;
            orgY = coordY;
            orgZ = coordZ;

            line.v = stepSpdL * velRatio;
            line.a = stepAccL * accRatio;
            s = line.v * line.v / line.a;

            if (s < line.s) {
                line.t1 = line.v / line.a;
                line.t2 = (line.s - s) / line.v;
                line.T = 2 * line.t1 + line.t2;
            } else {
                v = sqrt(line.a * line.s);
                line.t1 = v / line.a;
                line.t2 = 0;
                line.T =  2 * line.t1;
                line.v = v;
            }
            T1 = line.t1;
            T2 = line.t2;
            T = line.T;

            line.finishFlag = false;
            planFlag = true;
            periodCount = 0;
            catchFlag = false;
        }
        if (!line.finishFlag) {
            getSpeedLine();
        } else {
            finishFlag = true;
            planFlag = false;
            coordAxisP = r - coordAxisB;
            periodCount = 0;
            stepLastL = 0;
            step1 = 0;
            step2 = 0;
            step3 = 0;
            step4 = 0;
        }
    }
    if (finishFlag && !pauseFlag) {
        finishFlag = false;
        pauseFlag = true;
    }
    if(pauseFlag) {
        step1 = 0;
        step2 = 0;
        step3 = 0;
        step4 = 0;
        pauseCount ++;
        if(pauseCount > (pauseTime * 500.0 / 20)) {
            catchFlag = true;
        }
        if(pauseCount > (pauseTime * 1000.0 / 20)) {
            finishFlag = true;
            pauseCount = 0;
            pauseFlag = false;
        }
    }
}

/*********************************************************************************************************
** Function name:       movlINC
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::movlINC(float dx, float dy, float dz,  float dr)
{
    float s,v;

    if (!finishFlag && !pauseFlag) {
        if (!planFlag) {
            sendServoP = coordAxisP + dr;

            line.s = sqrt(dx * dx + dy * dy + dz * dz);
            dirX = dx / line.s;
            dirY = dy / line.s;
            dirZ = dz / line.s;

            orgX = coordX;
            orgY = coordY;
            orgZ = coordZ;

            line.v = stepSpdL * velRatio;
            line.a = stepAccL * accRatio;
            s = line.v * line.v / line.a;

            if (s < line.s) {
                line.t1 = line.v / line.a;
                line.t2 = (line.s - s) / line.v;
                line.T = 2 * line.t1 + line.t2;
            } else {
                v = sqrt(line.a * line.s);
                line.t1 = v / line.a;
                line.t2 = 0;
                line.T =  2 * line.t1;
                line.v = v;
            }
            T1 = line.t1;
            T2 = line.t2;
            T = line.T;

            line.finishFlag = false;
            planFlag = true;
            periodCount = 0;
        }
        if (!line.finishFlag) {
            getSpeedLine();
        } else {
            finishFlag = true;
            planFlag = false;
            coordAxisP = coordAxisP + dr;
            periodCount = 0;
            stepLastL = 0;
            step1 = 0;
            step2 = 0;
            step3 = 0;
            step4 = 0;
        }
    }
    if (finishFlag && !pauseFlag) {
        finishFlag = false;
        pauseFlag = true;
    }
    if(pauseFlag) {
        step1 = 0;
        step2 = 0;
        step3 = 0;
        step4 = 0;
        pauseCount ++;
        if(pauseCount > (pauseTime * 1000.0 / 20)) {
            finishFlag = true;
            pauseCount = 0;
            pauseFlag = false;
        }
    }
}

/*********************************************************************************************************
** Function name:       movj
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::movj(float b, float a1, float a2, float p)
{
    double x, v;

    if (!finishFlag && !pauseFlag) {
        if (!planFlag) {
            axisB.a = stepAccB * accRatio;
            axisH.a = stepAccH * accRatio;
            axisV.a = stepAccV * accRatio;
            axisP.a = stepAccH * accRatio;

            axisB.v = stepSpdB * velRatio;
            axisH.v = stepSpdH * velRatio;
            axisV.v = stepSpdV * velRatio;
            axisP.v = stepSpdP * velRatio;

            sendServoP = p;
            targetB = b;
            targetH = a1;
            targetV = a2;
            targetP = p;

            stepLastB = coordAxisB;
            stepLastH = coordAxisH;
            stepLastV = coordAxisV;
            stepLastP = coordAxisP;

            axisB.s = b - stepLastB;
            dirB = 1;

            if (axisB.s < 0) {
                axisB.s = - axisB.s;
                dirB = -1;
            }
            axisB.finishFlag = axisB.s < 1e-6;
            x = axisB.v * axisB.v / axisB.a;

            if (x<axisB.s) {
                axisB.t1 = axisB.v / axisB.a;
                axisB.t2 = (axisB.s - x) / axisB.v;
                axisB.T = 2 * axisB.t1 + axisB.t2;
            } else {
                v = sqrt(axisB.a * axisB.s);
                axisB.t1 = v / axisB.a;
                axisB.t2 = 0;
                axisB.T = 2 * axisB.t1;
            }
            T = axisB.T;

            axisH.s = a1 - stepLastH;
            dirH = 1;
            if (axisH.s < 0) {
                axisH.s = - axisH.s;
                dirH = -1;
            }
            axisH.finishFlag = axisH.s < 1e-6;
            x = axisH.v * axisH.v / axisH.a;
            if (x<axisH.s) {
                axisH.t1 = axisH.v / axisH.a;
                axisH.t2 = (axisH.s - x) / axisH.v;
               axisH.T = 2 * axisH.t1 + axisH.t2;
            } else {
                v = sqrt(axisH.a * axisH.s);
                axisH.t1 = v / axisH.a;
                axisH.t2 = 0;
               axisH.T = 2 * axisH.t1;
            }
            if (axisH.T > T) {
                T = axisH.T;
            }

            axisV.s = a2 - stepLastV;
            dirV = 1;
            if (axisV.s < 0) {
                axisV.s = - axisV.s;
                dirV = -1;
            }
            axisV.finishFlag = axisV.s < 1e-6;
            x = axisV.v * axisV.v / axisV.a;
            if (x<axisV.s) {
                axisV.t1 = axisV.v / axisV.a;
                axisV.t2 = (axisV.s - x) / axisV.v;
                axisV.T = 2 * axisV.t1 + axisV.t2;
            } else {
                v = sqrt(axisV.a * axisV.s);
                axisV.t1 = v / axisV.a;
                axisV.t2 = 0;
                axisV.T = 2 * axisV.t1;
            }
            if (axisV.T > T) {
                T =  axisV.T;
            }

            axisP.s = p - stepLastP;
            dirP = 1;
            if (axisP.s < 0) {
                axisP.s = - axisP.s;
                dirP = -1;
            }
            axisP.finishFlag = axisP.s < 1e-6;
            x = axisP.v * axisP.v / axisP.a;
            if (x<axisP.s) {
                axisP.t1 = axisP.v / axisP.a;
                axisP.t2 = (axisP.s - x) / axisP.v;
                axisP.T = 2 * axisP.t1 + axisP.t2;
            } else {
                v = sqrt(axisP.a*axisP.s);
                axisP.t1 = v/axisP.a;
                axisP.t2 = 0;
                axisP.T = 2 * axisP.t1;
            }
            if (axisP.T > T) {
                T = axisP.T;
            }
            if (2 * axisB.s / T <= axisB.v) {
                axisB.t1 = T / 2;
                axisB.t2 = 0;
                axisB.a = axisB.s / axisB.t1 / axisB.t1;
            } else {
                axisB.t1 = T - axisB.s / axisB.v;
                axisB.t2 = T - 2 * axisB.t1;
                axisB.a = axisB.v / axisB.t1;
            }
            axisB.v = axisB.a * axisB.t1;

            if (2*axisH.s/T <= axisH.v) {
                axisH.t1 = T / 2;
                axisH.t2 = 0;
                axisH.a = axisH.s / axisH.t1 / axisH.t1;
            } else {
                axisH.t1 = T - axisH.s / axisH.v;
                axisH.t2 = T - 2 * axisH.t1;
                axisH.a = axisH.v / axisH.t1;
            }
            axisH.v = axisH.a * axisH.t1;

            if (2 * axisV.s / T <= axisV.v) {
                axisV.t1 = T / 2;
                axisV.t2 = 0;
                axisV.a = axisV.s / axisV.t1 / axisV.t1;
            } else {
                axisV.t1 = T - axisV.s / axisV.v;
                axisV.t2 = T - 2 * axisV.t1;
                axisV.a = axisV.v / axisV.t1;
            }

            axisV.v = axisV.a * axisV.t1;

            if (2 * axisP.s / T <= axisP.v) {
                axisP.t1 = T / 2;
                axisP.t2 = 0;
                axisP.a = axisP.s / axisP.t1 / axisP.t1;
            } else {
                axisP.t1 = T - axisP.s / axisP.v;
                axisP.t2 = T - 2 * axisP.t1;
                axisP.a = axisP.v / axisP.t1;
            }
            axisP.v = axisP.a * axisP.t1;

            planFlag = true;
            catchFlag = false;
            periodCount = 0;
        }
        if (!(axisB.finishFlag && axisH.finishFlag && axisV.finishFlag && axisP.finishFlag)) {
            getSpeedJoint();
        } else {
            finishFlag = true;
            periodCount = 0;
            stepLastL = 0;
            step1 = 0;
            step2 = 0;
            step3 = 0;
            step4 = 0;
        }
    }
    if (finishFlag && !pauseFlag) {
        finishFlag = false;
        pauseFlag = true;
    }
    if(pauseFlag) {
        step1 = 0;
        step2 = 0;
        step3 = 0;
        step4 = 0;
        pauseCount ++;
        if(pauseCount > (pauseTime * 500.0 / 20)) {
            catchFlag = true;
        }
        if(pauseCount > (pauseTime * 1000.0 / 20)) {
            pauseCount = 0;
            finishFlag = true;
            planFlag = false;
            pauseFlag = false;
        }
    }
}

/*********************************************************************************************************
** Function name:       movjXYZ
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::movjXYZ(float x, float y, float z, float r)
{
    double s;

    if (!finishFlag && !pauseFlag) {
        if (!planFlag) {

            float h, v, b, p;
            float a1, a2;

            float l1, l2;
            float distV, distH, dist;
            float theta, theta1, theta2;

            axisB.a = stepAccB * accRatio;
            axisH.a = stepAccH * accRatio;
            axisV.a = stepAccV * accRatio;
            axisP.a = stepAccH * accRatio;

            axisB.v = stepSpdB * velRatio;
            axisH.v = stepSpdH * velRatio;
            axisV.v = stepSpdV * velRatio;
            axisP.v = stepSpdP * velRatio;

            h = sqrt(x * x + y * y);
            v = z;
            b = atan2(y, x) / PI * 180;
            p = r - b;

            sendServoP = p;
            l1 = arm1Len;
            l2 = arm2Len;
            distV = v - baseV + linkBlockV;
            distH = h - baseH - linkBlockH;
            dist = sqrt(distH * distH + distV * distV);
            theta = atan(distV / distH);
            if(fabs((l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist)) > 1) {
                finishFlag = true;
                planFlag = false;
                step1 = 0;
                step2 = 0;
                step3 = 0;
                step4 = 0;
                return;
            }
            theta1 = acos((l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist)) + theta;
            theta2 = atan2((l1 * sin(theta1) - distV), (distH - l1 * cos(theta1)));

            solvedH = 90 - theta1 / PI * 180;
            solvedV = theta2 / PI * 180;

            solvedB = b;
            solvedP = p;

            if (solvedH < ARM_A1_MIN || solvedH > ARM_A1_MAX || solvedV < ARM_A2_MIN || solvedV > ARM_A2_MAX) {
                finishFlag = true;
                planFlag = false;
                step1 = 0;
                step2 = 0;
                step3 = 0;
                step4 = 0;
                return;
            }
            a1 = solvedH;
            a2 = solvedV;

            targetB = b;
            targetH = a1;
            targetV = a2;
            targetP = p;

            stepLastB = coordAxisB;
            stepLastH = coordAxisH;
            stepLastV = coordAxisV;
            stepLastP = coordAxisP;

            axisB.s = b - stepLastB;
            dirB = 1;

            if (axisB.s < 0) {
                axisB.s = - axisB.s;
                dirB = -1;
            }
            axisB.finishFlag = axisB.s < 1e-6;
            s = axisB.v * axisB.v / axisB.a;

            if (s<axisB.s) {
                axisB.t1 = axisB.v / axisB.a;
                axisB.t2 = (axisB.s-s)/axisB.v;
                axisB.T = 2 * axisB.t1 + axisB.t2;
            } else {
                v = sqrt(axisB.a * axisB.s);
                axisB.t1 = v / axisB.a;
                axisB.t2 = 0;
                axisB.T = 2 * axisB.t1;
            }
            T = axisB.T;

            axisH.s = a1 - stepLastH;
            dirH = 1;
            if (axisH.s < 0) {
                axisH.s = - axisH.s;
                dirH = -1;
            }
            axisH.finishFlag = axisH.s < 1e-6;
            s = axisH.v * axisH.v / axisH.a;
            if (s<axisH.s) {
                axisH.t1 = axisH.v / axisH.a;
                axisH.t2 = (axisH.s-s)/axisH.v;
                axisH.T = 2 * axisH.t1 + axisH.t2;
            } else {
                v = sqrt(axisH.a * axisH.s);
                axisH.t1 = v / axisH.a;
                axisH.t2 = 0;
                axisH.T = 2 * axisH.t1;
            }
            if (axisH.T > T) {
                T = axisH.T;
            }

            axisV.s = a2 - stepLastV;
            dirV = 1;
            if (axisV.s < 0) {
                axisV.s = - axisV.s;
                dirV = -1;
            }
            axisV.finishFlag = axisV.s < 1e-6;
            s = axisV.v * axisV.v / axisV.a;
            if (s<axisV.s) {
                axisV.t1 = axisV.v / axisV.a;
                axisV.t2 = (axisV.s-s)/axisV.v;
                axisV.T = 2 * axisV.t1 + axisV.t2;
            } else {
                v = sqrt(axisV.a * axisV.s);
                axisV.t1 = v / axisV.a;
                axisV.t2 = 0;
                axisV.T = 2 * axisV.t1;
            }
            if (axisV.T > T) {
                T =  axisV.T;
            }

            axisP.s = p - stepLastP;
            dirP = 1;
            if (axisP.s < 0) {
                axisP.s = - axisP.s;
                dirP = -1;
            }
            axisP.finishFlag = axisP.s < 1e-6;
            s = axisP.v * axisP.v / axisP.a;
            if (s<axisP.s) {
                axisP.t1 = axisP.v / axisP.a;
                axisP.t2 = (axisP.s-s)/axisP.v;
               axisP.T = 2 * axisP.t1 + axisP.t2;
            } else {
                v = sqrt(axisP.a*axisP.s);
                axisP.t1 = v/axisP.a;
                axisP.t2 = 0;
               axisP.T = 2 * axisP.t1;
            }
            if (axisP.T > T) {
                T = axisP.T;
            }
            if (2 * axisB.s / T <= axisB.v) {
                axisB.t1 = T / 2;
                axisB.t2 = 0;
                axisB.a = axisB.s / axisB.t1 / axisB.t1;
            } else {
                axisB.t1 = T - axisB.s / axisB.v;
                axisB.t2 = T - 2 * axisB.t1;
                axisB.a = axisB.v / axisB.t1;
            }
            axisB.v = axisB.a * axisB.t1;

            if (2*axisH.s/T <= axisH.v) {
                axisH.t1 = T / 2;
                axisH.t2 = 0;
                axisH.a = axisH.s / axisH.t1 / axisH.t1;
            } else {
                axisH.t1 = T - axisH.s / axisH.v;
                axisH.t2 = T - 2 * axisH.t1;
                axisH.a = axisH.v / axisH.t1;
            }
            axisH.v = axisH.a * axisH.t1;

            if (2 * axisV.s / T <= axisV.v) {
                axisV.t1 = T / 2;
                axisV.t2 = 0;
                axisV.a = axisV.s / axisV.t1 / axisV.t1;
            } else {
                axisV.t1 = T - axisV.s / axisV.v;
                axisV.t2 = T - 2 * axisV.t1;
                axisV.a = axisV.v / axisV.t1;
            }

            axisV.v = axisV.a * axisV.t1;

            if (2 * axisP.s / T <= axisP.v) {
                axisP.t1 = T / 2;
                axisP.t2 = 0;
                axisP.a = axisP.s / axisP.t1 / axisP.t1;
            } else {
                axisP.t1 = T - axisP.s / axisP.v;
                axisP.t2 = T - 2 * axisP.t1;
                axisP.a = axisP.v / axisP.t1;
            }
            axisP.v = axisP.a * axisP.t1;

            planFlag = true;
            periodCount = 0;
            catchFlag = false;
        }
        if (!(axisB.finishFlag && axisH.finishFlag && axisV.finishFlag && axisP.finishFlag)) {
            getSpeedJoint();
        } else {
            finishFlag = true;
            periodCount = 0;
            stepLastL = 0;
            step1 = 0;
            step2 = 0;
            step3 = 0;
            step4 = 0;
        }
    }
    if (finishFlag && !pauseFlag) {
        finishFlag = false;
        pauseFlag = true;
    }
    if(pauseFlag) {
        step1 = 0;
        step2 = 0;
        step3 = 0;
        step4 = 0;
        pauseCount ++;
        if(pauseCount > (pauseTime * 500.0 / 20)) {
            catchFlag = true;
        }
        if(pauseCount > (pauseTime * 1000.0 / 20)) {
            pauseCount = 0;
            finishFlag = true;
            planFlag = false;
            pauseFlag = false;
        }
    }
}

/*********************************************************************************************************
** Function name:       movjINC
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::movjINC(float db, float da1, float da2,  float dp)
{
    double x, v;

    if (!finishFlag && !pauseFlag) {
        if (!planFlag) {
            axisB.a = stepAccB * accRatio;
            axisH.a = stepAccH * accRatio;
            axisV.a = stepAccV * accRatio;
            axisP.a = stepAccH * accRatio;

            axisB.v = stepSpdB * velRatio;
            axisH.v = stepSpdH * velRatio;
            axisV.v = stepSpdV * velRatio;
            axisP.v = stepSpdP * velRatio;

            sendServoP = coordAxisP+dp;
            targetB = coordAxisB+db;
            targetH = coordAxisH+da1;
            targetV = coordAxisV+da2;
            targetP = coordAxisP+dp;

            stepLastB = coordAxisB;
            stepLastH = coordAxisH;
            stepLastV = coordAxisV;
            stepLastP = coordAxisP;

            axisB.s = dp;
            dirB = 1;
            if (axisB.s < 0) {
                axisB.s = - axisB.s;
                dirB = -1;
            }
            axisB.finishFlag = axisB.s < 1e-6;
            x = axisB.v * axisB.v / axisB.a;

            if (x<axisB.s) {
                axisB.t1 = axisB.v / axisB.a;
                axisB.t2 = (axisB.s - x) / axisB.v;
                axisB.T = 2 * axisB.t1 + axisB.t2;
            } else {
                v = sqrt(axisB.a * axisB.s);
                axisB.t1 = v / axisB.a;
                axisB.t2 = 0;
                axisB.T = 2 * axisB.t1;
            }
            T = axisB.T;

            axisH.s = da1;
            dirH = 1;
            if (axisH.s < 0) {
                axisH.s = - axisH.s;
                dirH = -1;
            }
            axisH.finishFlag = axisH.s < 1e-6;
            x = axisH.v * axisH.v / axisH.a;
            if (x<axisH.s) {
                axisH.t1 = axisH.v / axisH.a;
                axisH.t2 = (axisH.s - x) / axisH.v;
                axisH.T = 2 * axisH.t1 + axisH.t2;
            } else {
                v = sqrt(axisH.a * axisH.s);
                axisH.t1 = v / axisH.a;
                axisH.t2 = 0;
                axisH.T = 2 * axisH.t1;
            }
            if (axisH.T > T) {
                T = axisH.T;
            }

            axisV.s = da2;
            dirV = 1;
            if (axisV.s < 0) {
                axisV.s = - axisV.s;
                dirV = -1;
            }
            axisV.finishFlag = axisV.s < 1e-6;
            x = axisV.v * axisV.v / axisV.a;
            if (x<axisV.s) {
                axisV.t1 = axisV.v / axisV.a;
                axisV.t2 = (axisV.s - x) / axisV.v;
                axisV.T = 2 * axisV.t1 + axisV.t2;
            } else {
                v = sqrt(axisV.a * axisV.s);
                axisV.t1 = v / axisV.a;
                axisV.t2 = 0;
                axisV.T = 2 * axisV.t1;
            }
            if (axisV.T > T) {
                T =  axisV.T;
            }

            axisP.s = dp;
            dirP = 1;
            if (axisP.s < 0) {
                axisP.s = - axisP.s;
                dirP = -1;
            }
            axisP.finishFlag = axisP.s < 1e-6;
            x = axisP.v * axisP.v / axisP.a;
            if (x<axisP.s) {
                axisP.t1 = axisP.v / axisP.a;
                axisP.t2 = (axisP.s - x) / axisP.v;
                axisP.T = 2 * axisP.t1 + axisP.t2;
            } else {
                v = sqrt(axisP.a*axisP.s);
                axisP.t1 = v/axisP.a;
                axisP.t2 = 0;
                axisP.T = 2 * axisP.t1;
            }
            if (axisP.T > T) {
                T = axisP.T;
            }
            if (2 * axisB.s / T <= axisB.v) {
                axisB.t1 = T / 2;
                axisB.t2 = 0;
                axisB.a = axisB.s / axisB.t1 / axisB.t1;
            } else {
                axisB.t1 = T - axisB.s / axisB.v;
                axisB.t2 = T - 2 * axisB.t1;
                axisB.a = axisB.v / axisB.t1;
            }
            axisB.v = axisB.a * axisB.t1;

            if (2*axisH.s/T <= axisH.v) {
                axisH.t1 = T / 2;
                axisH.t2 = 0;
                axisH.a = axisH.s / axisH.t1 / axisH.t1;
            } else {
                axisH.t1 = T - axisH.s / axisH.v;
                axisH.t2 = T - 2 * axisH.t1;
                axisH.a = axisH.v / axisH.t1;
            }
            axisH.v = axisH.a * axisH.t1;

            if (2 * axisV.s / T <= axisV.v) {
                axisV.t1 = T / 2;
                axisV.t2 = 0;
                axisV.a = axisV.s / axisV.t1 / axisV.t1;
            } else {
                axisV.t1 = T - axisV.s / axisV.v;
                axisV.t2 = T - 2 * axisV.t1;
                axisV.a = axisV.v / axisV.t1;
            }
            axisV.v = axisV.a * axisV.t1;

            if (2 * axisP.s / T <= axisP.v) {
                axisP.t1 = T / 2;
                axisP.t2 = 0;
                axisP.a = axisP.s / axisP.t1 / axisP.t1;
            } else {
                axisP.t1 = T - axisP.s / axisP.v;
                axisP.t2 = T - 2 * axisP.t1;
                axisP.a = axisP.v / axisP.t1;
            }
            axisP.v = axisP.a * axisP.t1;

            planFlag = true;
            periodCount = 0;
        }

        if (!(axisB.finishFlag && axisH.finishFlag && axisV.finishFlag && axisP.finishFlag)) {
            getSpeedJoint();
        } else {
            finishFlag = true;
            periodCount = 0;
            stepLastL = 0;
            step1 = 0;
            step2 = 0;
            step3 = 0;
            step4 = 0;
        }
    }
    if (finishFlag && !pauseFlag) {
        finishFlag = false;
        pauseFlag = true;
    }
    if(pauseFlag) {
        step1 = 0;
        step2 = 0;
        step3 = 0;
        step4 = 0;
        pauseCount ++;
        if(pauseCount > (pauseTime * 1000.0 / 20)) {
            pauseCount = 0;
            finishFlag = true;
            planFlag = false;
            pauseFlag = false;
        }
    }
}

/*********************************************************************************************************
** Function name:       getSpeedLine
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::getSpeedLine()
{
    periodCount++;
    t0 = periodCount*periodTime;

    if (t0 <= T1) {
        speedLine = line.a * t0;
    } else if(t0 <= T1 + T2) {
        speedLine = line.v;
    } else {
        speedLine = line.v - line.a * (t0 - T1 - T2);
    }

    if (!line.finishFlag) {
        if (t0 >= T) {
            speedLine = (line.s - stepLastL) / periodTime;
            stepLastL = line.s;
            line.finishFlag = true;
        } else {
            stepLastL += speedLine * periodTime;
        }
    } else {
        speedLine = 0;
    }

    sendX = orgX + stepLastL * dirX;
    sendY = orgY + stepLastL * dirY;
    sendZ = orgZ + stepLastL * dirZ;

    inverse(sendX, sendY, sendZ, coordR);
}

/*********************************************************************************************************
** Function name:       getSpeedJoint
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::getSpeedJoint()
{
    periodCount++;
    t0 = periodCount*periodTime;

    T1 = axisB.t1;
    T2 = axisB.t2;
    if (t0 <= T1) {
        speedB = axisB.a * t0;
    } else if(t0 <= T1 + T2) {
        speedB = axisB.v;
    } else {
        speedB = axisB.v - axisB.a * (t0 - T1 - T2);
    }

    T1 = axisH.t1;
    T2 = axisH.t2;
    if (t0 <= T1) {
        speedH = axisH.a * t0;
    } else if(t0 <= T1 + T2) {
        speedH = axisH.v;
    } else {
        speedH = axisH.v - axisH.a * (t0 - T1 - T2);
    }

    T1 = axisV.t1;
    T2 = axisV.t2;
    if (t0 <= T1) {
        speedV = axisV.a * t0;
    } else if(t0 <= T1 + T2) {
        speedV = axisV.v;
    } else {
        speedV = axisV.v - axisV.a * (t0 - T1 - T2);
    }

    T1 = axisP.t1;
    T2 = axisP.t2;
    if (t0 <= T1) {
        speedP = axisP.a * t0;
    } else if(t0 <= T1 + T2) {
        speedP = axisP.v;
    } else {
        speedP = axisP.v - axisP.a * (t0 - T1 - T2);
    }

    speedB = speedB*dirB;
    speedH = speedH*dirH;
    speedV = speedV*dirV;
    speedP = speedP*dirP;

    if (!axisB.finishFlag) {
        if (t0 >= T) {
            speedB = (targetB - stepLastB) / periodTime;
            stepLastB=targetB;
            axisB.finishFlag = true;
        } else {
            stepLastB += speedB * periodTime;
        }
    } else {
        speedB = 0;
    }

    if (!axisH.finishFlag) {
        if (t0 >= T) {
            speedH = (targetH - stepLastH) / periodTime;
            stepLastH = targetH;
            axisH.finishFlag = true;
        } else {
            stepLastH += speedH * periodTime;
        }
    } else {
        speedH = 0;
    }

    if (!axisV.finishFlag) {
        if (t0 >= T) {
            speedV = (targetV - stepLastV) / periodTime;
            stepLastV = targetV;
            axisV.finishFlag = true;
        } else {
            stepLastV += speedV * periodTime;
        }
    } else {
        speedV = 0;
    }

    if (!axisP.finishFlag) {
        if (t0 >= T) {
            speedP = (targetP - stepLastP) / periodTime;
            stepLastP=targetP;
            axisP.finishFlag = true;
        } else {
            stepLastP += speedP * periodTime;
        }
    } else {
        speedP = 0;
    }
    writePulse(speedB, speedH, speedV, speedP);
}

/*********************************************************************************************************
** Function name:       writePulse
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::writePulse(float b, float h, float v, float p)
{
    float coordNextB;
    float coordNextH;
    float coordNextV;
    float coordNextP;
    float armStrech;
    CurrentCoordinateType coordNext;

    coordNextB = coordAxisB + b * periodTime;
    coordNextH = coordAxisH + h * periodTime;
    coordNextV = coordAxisV + v * periodTime;
    coordNextP = coordAxisP + p * periodTime;

    getNextPositionXYZR(coordNextB, coordNextH, coordNextV, coordNextP, &coordNext);

    armStrech = sqrt(coordNext.x * coordNext.x + coordNext.y * coordNext.y);

    if ((coordNextB<BASE_ROTATION_MIN && b < 0) || (coordNextB>BASE_ROTATION_MAX && b > 0)
        || (coordNextH<ARM_A1_MIN && h < 0) || (coordNextH>ARM_A1_MAX && h > 0)
        || (coordNextV<ARM_A2_MIN && v < 0) || (coordNextV>ARM_A2_MAX && v > 0)
        || (coordNextP<HAND_ROTATION_MIN && p < 0) || (coordNextP>HAND_ROTATION_MAX && p > 0)
        || (((armStrech > ARM_STRETCH_MAX + 1)
        || (armStrech < ARM_STRETCH_MIN - 1)
        || (coordNext.z > ARM_HEIGHT_MAX + 1)
        || (90 + coordNextH - coordNextV > ARM_A12_MAX)) && !limitFlag)
        ) {
        step1 = 0;
        step2 = 0;
        step3 = 0;
        step4 = 0;
        if (!firstLimitFlag)
        {
            limitFlag = true;
            firstLimitFlag = true;
        }

        finishFlag = true;
        planFlag = false;
        return;
    }
    else{
        coordAxisB += b * periodTime;
        coordAxisH += h * periodTime;
        coordAxisV += v * periodTime;
        coordAxisP += p * periodTime;
        if (!((armStrech > ARM_STRETCH_MAX + 1)
            || (armStrech < ARM_STRETCH_MIN - 1)
            || (coordNext.z > ARM_HEIGHT_MAX + 1)
            || (90 + coordNextH - coordNextV > ARM_A12_MAX)))
        {
            limitFlag = false;
        }

    }
    remainPulseB += b * periodTime / stepAngle * subdiv1 * gearRatio1;
    int Np = (int)remainPulseB;
    if (fabs(remainPulseB) < 1) {
        step1 = 0;
    } else {
        remainPulseB -= Np;
        sendPulseSumB += Np;
        step1 = stepDir1 * Np;
    }
    remainPulseH += h * periodTime / stepAngle * subdiv2 * gearRatio2;
    Np = (int)remainPulseH;
    if (fabs(remainPulseH) < 1) {
        step2 = 0;
    } else {
        remainPulseH -= Np;
        sendPulseSumH += Np;
        step2 = stepDir2 * Np;
    }
    remainPulseV += v * periodTime / stepAngle * subdiv3 * gearRatio3;
    Np = (int)remainPulseV;
    if (fabs(remainPulseV) < 1) {
        step3 = 0;
    } else {
        remainPulseV -= Np;
        sendPulseSumV += Np;
        step3 = stepDir3 * Np;
    }
    remainPulseP += p * periodTime / stepAngle;
    Np = (int)remainPulseP;
    if (fabs(remainPulseP) < 1) {
        step4 = 0;
    } else {
        remainPulseP -= Np;
        sendPulseSumP += Np;
        step4 = -Np;
    }
    sendSpeedBSum = sendPulseSumB / subdiv1 / gearRatio1 * stepAngle;
    sendSpeedHSum = sendPulseSumH / subdiv2 / gearRatio2 * stepAngle;
    sendSpeedVSum = sendPulseSumV / subdiv3 / gearRatio3 * stepAngle;
    sendSpeedPSum = sendPulseSumP * stepAngle;
}

/*********************************************************************************************************
** Function name:       writeGripperPulse
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::writeGripperPulse(float g)
{
    coordAxisG += g * periodTime;
}

/*********************************************************************************************************
** Function name:       inverse
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::inverse(float x, float y, float z, float r)
{
    float sendA1, sendA2, sendB, sendP;
    float h, v, b, p;
    float l1, l2;
    float distV, distH, dist;
    float theta, theta1, theta2;

    h = sqrt(x * x + y * y);
    v = z;
    b = atan2(y, x) / PI * 180;
    p = r - b;

    l1 = arm1Len;
    l2 = arm2Len;
    distV = v - baseV + linkBlockV;
    distH = h - baseH - linkBlockH;
    dist = sqrt(distH * distH + distV * distV);
    theta = atan(distV / distH);
    theta1 = acos((l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist)) + theta;
    theta2 = atan2((l1 * sin(theta1) - distV), (distH -l1 * cos(theta1)));

    solvedH = 90 - theta1 / PI * 180;
    solvedV = theta2 / PI * 180;

    solvedB = b;
    solvedP = p;

    sendA1 = (solvedH - coordAxisH) / periodTime;
    sendA2 = (solvedV - coordAxisV) / periodTime;
    sendB = (solvedB - coordAxisB) / periodTime;
    sendP = (solvedP - coordAxisP) / periodTime;

    writePulse(sendB, sendA1, sendA2, sendP);
}

static float vbg = 0;

/*********************************************************************************************************
** Function name:       planSpeed
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::planSpeed(float vs, float ve, float vm, float dx, float dy,float dz,float stopflag,float isLaserOn,float state)
{
    if(!finishFlag) {
        if (!planFlag) {
            if (stopflag == 1) {
                stepper.laserOff();
                finishFlag = true;
                planFlag = true;;
            }
            if (state == 0) {
                seg.accelat = accWrite;
                seg.interp_time.AT = accWrite * 0.02f;
                seg.gCode.z = dz;
            } else if(state == 1) {
                if(isLaserOn == 1) {
                    seg.accelat = accWrite;
                    seg.interp_time.AT = accWrite * 0.02f;
                } else if(isLaserOn == 0) {
                    seg.accelat = 100 * accWrite;
                    seg.interp_time.AT = 100 * accWrite * 0.02f;
                }
                seg.gCode.z = 0;
            }
            orgX = coordX;
            orgY = coordY;
            orgZ = coordZ;
            seg.gCode.x = dx;
            seg.gCode.y = dy;

            seg.dist = (float)sqrt(dx * dx + dy * dy + dz * dz);

            seg.interp_time.vel = vm;
            seg.ve = ve;
            seg.vb = vbg;

            seg.axis_seg[0] = 0;
            seg.axis_seg[1] = 0;
            seg.axis_seg[2] = 0;
            line_interp_plan(&seg);
            planFlag = true;
        }
        if(state == 1) {
            if(isLaserOn == 1) {
                stepper.laserOn();
            } else if(isLaserOn == 0) {
                stepper.laserOff();
            }
        }
        if (!finishFlag) {
            line_interp_deal(&seg);

            inverse(sendX, sendY, sendZ, coordR);
        }
    }
}

/*********************************************************************************************************
** Function name:       getCurrentCoord
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::getCurrentCoord()
{
    coorGet.x = coordX;
    coorGet.y = coordY;
    coorGet.z = coordZ;
    coorGet.rHead = coordR;
    coorGet.baseAngle = coordAxisB;
    coorGet.longArmAngle = coordAxisH;
    coorGet.shortArmAngle = coordAxisV;
    coorGet.pawAngle = coordAxisP;
    coorGet.gripperAngle = coordAxisG;
}

/*********************************************************************************************************
** Function name:       single
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::single(int state)
{
    switch (state) {
        case AP_DOWN:
             singleVelMax = singleVelMaxB * singleVelRatio;
             singleAcc = singleAccB;
             singlePos();
             writePulse(singleVel, 0, 0, 0);
             lastState = AP_DOWN;
            break;
        case AN_DOWN:
             singleVelMax = singleVelMaxB * singleVelRatio;
             singleAcc = singleAccB;
             singleNeg();
             writePulse(singleVel, 0, 0, 0);
             lastState = AN_DOWN;
            break;
        case BP_DOWN:
            singleVelMax = singleVelMaxH * singleVelRatio;
            singleAcc = singleAccH;
            singlePos();
            writePulse(0, singleVel, 0, 0);
            lastState = BP_DOWN;
            break;
        case BN_DOWN:
            singleVelMax = singleVelMaxH * singleVelRatio;
            singleAcc = singleAccH;
            singleNeg();
            writePulse(0, singleVel, 0, 0);
            lastState = BN_DOWN;
            break;
        case CP_DOWN:
            singleVelMax = singleVelMaxV * singleVelRatio;
            singleAcc = singleAccV;
            singlePos();
            writePulse(0, 0, singleVel, 0);
            lastState = CP_DOWN;
            break;
        case CN_DOWN:
            singleVelMax = singleVelMaxV * singleVelRatio;
            singleAcc = singleAccV;
            singleNeg();
            writePulse(0, 0, singleVel, 0);
            lastState = CN_DOWN;
            break;
        case DP_DOWN:
            singleVelMax = singleVelMaxP * singleVelRatio;
            singleAcc = singleAccP;
            singlePos();
            writePulse(0, 0, 0, singleVel);
            lastState = DP_DOWN;
            break;
        case DN_DOWN:
            singleVelMax = singleVelMaxP * singleVelRatio;
            singleAcc = singleAccP;
            singleNeg();
            writePulse(0, 0, 0, singleVel);
            lastState = DN_DOWN;
            break;
        case EP_DOWN:
            singleVelMax = singleVelMaxP * singleVelRatio;
            singleAcc = singleAccP;
            singlePos();
            writeGripperPulse(singleVel);
            lastState = EP_DOWN;
            break;
        case EN_DOWN:
            singleVelMax = singleVelMaxP * singleVelRatio;
            singleAcc = singleAccP;
            singleNeg();
            writeGripperPulse(singleVel);
            lastState = EN_DOWN;
            break;
        case  IDEL:
            singleStop();
            break;
        default:
            break;
    }
}

/*********************************************************************************************************
** Function name:       singlePos
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::singlePos()
{
    if (singleVel<singleVelMax) {
        singleVel += singleAcc * periodTime;
    } else {
        singleVel = singleVelMax;
    }
    finishFlag = true;
}

/*********************************************************************************************************
** Function name:       singleNeg
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::singleNeg()
{
    if (singleVel > -singleVelMax) {
        singleVel -= singleAcc * periodTime;
    } else {
        singleVel = -singleVelMax;
    }
    finishFlag = true;
}

/*********************************************************************************************************
** Function name:       singleStop
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::singleStop()
{
    if(singleVel == 0) {
        finishFlag = true;
        return;
    }
    if (lastState == AP_DOWN || lastState == BP_DOWN || lastState == CP_DOWN || lastState == DP_DOWN || lastState == EP_DOWN) {
        if (singleVel > 0) {
            singleVel -= singleAcc * periodTime;
        } else {
            singleVel = 0;
            finishFlag = true;
        }
    } else if (lastState == AN_DOWN || lastState == BN_DOWN || lastState == CN_DOWN || lastState == DN_DOWN || lastState == EN_DOWN) {
        if (singleVel < 0) {
            singleVel += singleAcc * periodTime;
        } else {
            singleVel = 0;
            finishFlag = true;
        }
    } else {
        finishFlag = true;
    }

    switch (lastState) {
        case AP_DOWN:
            writePulse(singleVel, 0, 0, 0);
            break;
        case AN_DOWN:
            writePulse(singleVel, 0, 0, 0);
            break;
        case BP_DOWN:
            writePulse(0, singleVel, 0, 0);
            break;
        case BN_DOWN:
            writePulse(0, singleVel, 0, 0);
            break;
        case CP_DOWN:
            writePulse(0, 0, singleVel, 0);
            break;
        case CN_DOWN:
            writePulse(0, 0, singleVel, 0);
            break;
        case DP_DOWN:
            writePulse(0, 0, 0, singleVel);
            break;
        case DN_DOWN:
            writePulse(0, 0, 0, singleVel);
            break;
        case EP_DOWN:
            writeGripperPulse(singleVel);
            break;
        case EN_DOWN:
            writeGripperPulse(singleVel);
            break;
        default:
            break;
    }
}

/*********************************************************************************************************
** Function name:       singleXY
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::singleXY(int state)
{
    switch (state) {
        case AP_DOWN:
            singleVelMax = singleVelMaxX * singleVelRatio;
            singleAcc = singleAccX;
            singlePos();
            coordX = coordX + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            lastState = AP_DOWN;
            break;
        case AN_DOWN:
            singleVelMax = singleVelMaxX * singleVelRatio;
            singleAcc = singleAccX;
            singleNeg();
            coordX = coordX + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            lastState = AN_DOWN;
            break;
        case BP_DOWN:
            singleVelMax = singleVelMaxY * singleVelRatio;
            singleAcc = singleAccY;
            singlePos();
            coordY = coordY + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            lastState = BP_DOWN;
            break;
        case BN_DOWN:
            singleVelMax = singleVelMaxY * singleVelRatio;
            singleAcc = singleAccY;
            singleNeg();
            coordY = coordY + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            lastState = BN_DOWN;
            break;
        case CP_DOWN:
            singleVelMax = singleVelMaxZ * singleVelRatio;
            singleAcc = singleAccZ;
            singlePos();
            coordZ = coordZ + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            lastState = CP_DOWN;
            break;
        case CN_DOWN:
            singleVelMax = singleVelMaxZ * singleVelRatio;
            singleAcc = singleAccZ;
            singleNeg();
            coordZ = coordZ + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            lastState = CN_DOWN;
            break;
        case DP_DOWN:
            singleVelMax = singleVelMaxP * singleVelRatio;
            singleAcc = singleAccP;
            singlePos();
            coordR = coordR + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            lastState = DP_DOWN;
            break;
        case DN_DOWN:
            singleVelMax = singleVelMaxP * singleVelRatio;
            singleAcc = singleAccP;
            singleNeg();
            coordR = coordR + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            lastState = DN_DOWN;
            break;
        case EP_DOWN:
            singleVelMax = singleVelMaxP * singleVelRatio;
            singleAcc = singleAccP;
            singlePos();
            writeGripperPulse(singleVel);
            lastState = EP_DOWN;
            break;
        case EN_DOWN:
            singleVelMax = singleVelMaxP * singleVelRatio;
            singleAcc = singleAccP;
            singleNeg();
            writeGripperPulse(singleVel);
            lastState = EN_DOWN;
            break;
        case  IDEL:
            singleXYStop();
            break;
        default:
            break;
    }
}

/*********************************************************************************************************
** Function name:       singleXYStop
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::singleXYStop()
{
    if(singleVel == 0) {
        finishFlag = true;
        return;
    }

    if (lastState == AP_DOWN || lastState == BP_DOWN || lastState == CP_DOWN || lastState == DP_DOWN || lastState == EP_DOWN) {
        if (singleVel > 0) {
            singleVel -= singleAcc * periodTime;
        } else {
            singleVel = 0;
            finishFlag = true;
        }
    } else if (lastState == AN_DOWN || lastState == BN_DOWN || lastState == CN_DOWN || lastState == DN_DOWN || lastState == EN_DOWN) {
        if (singleVel < 0) {
            singleVel += singleAcc * periodTime;
        } else {
            singleVel = 0;
            finishFlag = true;
        }
    } else {
        finishFlag = true;
    }
    switch (lastState) {
        case AP_DOWN:
            coordX = coordX + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            break;
        case AN_DOWN:
            coordX = coordX + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            break;
        case BP_DOWN:
            coordY = coordY + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            break;
        case BN_DOWN:
            coordY = coordY + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            break;
        case CP_DOWN:
            coordZ = coordZ + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            break;
        case CN_DOWN:
            coordZ = coordZ + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            break;
        case DP_DOWN:
            coordR = coordR + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            break;
        case DN_DOWN:
            coordR = coordR + singleVel * periodTime;
            inverse(coordX, coordY, coordZ, coordR);
            break;
        case EP_DOWN:
            writeGripperPulse(singleVel);
            break;
        case EN_DOWN:
            writeGripperPulse(singleVel);
            break;
        default:
            break;
    }
}

/*********************************************************************************************************
** Function name:       absX
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
float Dobot::absX(float a)
{
    if (a<0) {
        return -a;
    }
    return a;
}

volatile float le;
volatile float li;
volatile float vst, vbt, vet;
volatile float sa = 0;
volatile float su = 0;
volatile float sde = 0;
volatile float sum = 0;
volatile float lrem = 0;
volatile int add = 0;
volatile int unfm = 0;
volatile int dec = 0;
volatile float vb = 0;
volatile float AT = 0;
int testcnt = 0;

/*********************************************************************************************************
** Function name:       line_interp_plan
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void Dobot::line_interp_plan(typ_interp_segment * interpl_segment)
{
    if (interpl_segment->vb>interpl_segment->interp_time.vel) {
        interpl_segment->vb = interpl_segment->interp_time.vel;
    }
    if (interpl_segment->ve>interpl_segment->interp_time.vel) {
        interpl_segment->ve = interpl_segment->interp_time.vel;
    }
    vst = (float)(interpl_segment->interp_time.vel*interpl_segment->interp_time.vel);
    vbt = (float)(interpl_segment->vb*interpl_segment->vb);
    vet = (float)(interpl_segment->ve*interpl_segment->ve);
    if (interpl_segment->dist > LINE_MIN_DIST) {
        le = (vst - 0.5f*vbt - 0.5f*vet) / interpl_segment->accelat;
        if (le <= interpl_segment->dist) {
            interpl_segment->vm = interpl_segment->interp_time.vel;
            interpl_segment->interp_time.term_unfm = ((interpl_segment->dist - le) / (interpl_segment->vm*INTERP_CYC)) + 0.5f;
            interpl_segment->interp_time.term_add = ((interpl_segment->vm - interpl_segment->vb) / interpl_segment->interp_time.AT) + 0.5f;
            interpl_segment->interp_time.term_dec = ((interpl_segment->vm - interpl_segment->ve) / interpl_segment->interp_time.AT) + 0.5f;
        } else {
            li = 0.5f*(vet - vbt) / interpl_segment->accelat;
            if (interpl_segment->dist <= absX(li)) {
                if (li >= 0) {
                    interpl_segment->vm = sqrt(2 * interpl_segment->accelat*interpl_segment->dist + vbt);
                    interpl_segment->ve = interpl_segment->vm;
                    interpl_segment->interp_time.term_unfm = 0;
                    interpl_segment->interp_time.term_add = ((interpl_segment->ve - interpl_segment->vb) / interpl_segment->interp_time.AT) + 0.5f;
                    interpl_segment->interp_time.term_dec = 0;
                } else {
                    interpl_segment->vm = sqrt(2 * interpl_segment->accelat*interpl_segment->dist + vet);
                    interpl_segment->vb = interpl_segment->vm;
                    interpl_segment->interp_time.term_unfm = 0;
                    interpl_segment->interp_time.term_add = 0;
                    interpl_segment->interp_time.term_dec = ((interpl_segment->vb - interpl_segment->ve) / interpl_segment->interp_time.AT) + 0.5f;
                }
            } else {
                if (interpl_segment->vb<interpl_segment->interp_time.vel*LDEGREE&& \
                    interpl_segment->ve<interpl_segment->interp_time.vel*LDEGREE) {
                    interpl_segment->vm = sqrt(interpl_segment->accelat*interpl_segment->dist + 0.5f*vbt + 0.5f*vet);
                    interpl_segment->interp_time.term_add = ((interpl_segment->vm - interpl_segment->vb) / interpl_segment->interp_time.AT) + 0.5f;
                    interpl_segment->interp_time.term_unfm = 0;
                    interpl_segment->interp_time.term_dec = ((interpl_segment->vm - interpl_segment->ve) / interpl_segment->interp_time.AT) + 0.5f;
                } else {
                    if (li >= 0) {
                        interpl_segment->vm = interpl_segment->ve;
                        interpl_segment->interp_time.term_add = (int)(((interpl_segment->ve - interpl_segment->vb) / interpl_segment->interp_time.AT) + 0.5f);
                        interpl_segment->interp_time.term_dec = 0;
                    } else {
                        interpl_segment->vm = interpl_segment->vb;
                        interpl_segment->interp_time.term_add = 0;
                        interpl_segment->interp_time.term_dec = (int)(((interpl_segment->vb - interpl_segment->ve) / interpl_segment->interp_time.AT) + 0.5f);
                    }
                }
            }
        }

        add = interpl_segment->interp_time.term_add;
        dec = interpl_segment->interp_time.term_dec;
        AT = interpl_segment->interp_time.AT;
        vb = interpl_segment->vb;


        sa = add*(2 * vb + (add - 1)*AT)*INTERP_CYC*0.5f;
        if (add>0) {
            sde = dec*(2 * vb + 2 * (add - 1)*AT - (dec - 1)*AT)*INTERP_CYC*0.5f;
        } else {
            sde = dec*(2 * vb - (dec + 1)*AT)*INTERP_CYC*0.5f;
        }

        interpl_segment->interp_time.term_unfm = INTERP_RECI*(interpl_segment->dist - sa - sde) / (vb + add*AT);
        unfm = interpl_segment->interp_time.term_unfm;
        if ((interpl_segment->interp_time.term_unfm + interpl_segment->interp_time.term_add + interpl_segment->interp_time.term_dec) == 0) {
            interpl_segment->interp_time.term_unfm = 1;
        }
        unfm = interpl_segment->interp_time.term_unfm;

        su = unfm*(vb + add*AT)*INTERP_CYC;
        sum = sa + su + sde;
        lrem = interpl_segment->dist - sum;
        interpl_segment->interp_time.term_rem = interpl_segment->interp_time.term_add + interpl_segment->interp_time.term_unfm \
            + interpl_segment->interp_time.term_dec;

        interpl_segment->interp_time.vDelta = INTERP_RECI*lrem / interpl_segment->interp_time.term_rem;
        if (abs(interpl_segment->interp_time.vDelta)>AT) {
            interpl_segment->interp_time.vDelta = AT;
        }

        if (add>0 && dec>0) {
            interpl_segment->ve = vb + add*AT - dec*AT + interpl_segment->interp_time.vDelta;
        } else if (add == 0 && dec == 0) {
            interpl_segment->ve = vb + interpl_segment->interp_time.vDelta;
        } else if (dec <= 0) {
            if (unfm <= 0) {
                interpl_segment->ve = vb + (add - 1)*AT + interpl_segment->interp_time.vDelta;
            } else {
                interpl_segment->ve = vb + (add)*AT + interpl_segment->interp_time.vDelta;
            }
        } else if (add <= 0) {
            interpl_segment->ve = vb - (dec)*AT + interpl_segment->interp_time.vDelta;
        }

        interpl_segment->interp_time.term_vel = interpl_segment->vb + interpl_segment->interp_time.vDelta;
    }
    else {
        interpl_segment->interp_time.term_cmd = 0;
        interpl_segment->interp_time.term_add = 0;
        interpl_segment->interp_time.term_rem = 0;
        interpl_segment->interp_time.term_unfm = 0;
        interpl_segment->interp_time.term_dec = 0;
        interpl_segment->interp_time.motion_stat = INP_MOTION_END;
        interpl_segment->interp_time.motion_stat_E = INP_MOTION_END;
    }
}

float nextRem = 0;

/*********************************************************************************************************
** Function name:       linear_vel_ctrl
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
int Dobot::linear_vel_ctrl(typ_interp_time * interp_time)
{
    if (interp_time->term_add>0) {
        interp_time->term_length = interp_time->term_vel*INTERP_CYC;
        interp_time->term_vel += interp_time->AT;
        interp_time->term_add--;
    } else if (interp_time->term_unfm>0) {
        interp_time->term_length = interp_time->term_vel*INTERP_CYC;
        interp_time->term_unfm--;
    } else if (interp_time->term_dec>0) {
        interp_time->term_vel -= interp_time->AT;
        interp_time->term_length = interp_time->term_vel*INTERP_CYC;
        interp_time->term_dec--;
    }

    if ((interp_time->term_add + interp_time->term_unfm + interp_time->term_dec) == 0) {
        return 1;
    }
    return 0;
}

/*********************************************************************************************************
** Function name:       line_interp_deal
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
int Dobot::line_interp_deal(typ_interp_segment * interpl_segment)
{
    int isSegDone = linear_vel_ctrl((typ_interp_time *)&interpl_segment->interp_time);
    interpl_segment->dist_out[0] = interpl_segment->interp_time.term_length*interpl_segment->gCode.x / interpl_segment->dist;
    interpl_segment->axis_seg[0] += interpl_segment->dist_out[0];

    interpl_segment->dist_out[1] = interpl_segment->interp_time.term_length*interpl_segment->gCode.y / interpl_segment->dist;
    interpl_segment->axis_seg[1] += interpl_segment->dist_out[1];

    interpl_segment->dist_out[2] = interpl_segment->interp_time.term_length*interpl_segment->gCode.z / interpl_segment->dist;
    interpl_segment->axis_seg[2] += interpl_segment->dist_out[2];

    sendX = orgX-interpl_segment->axis_seg[0];
    sendY = orgY+interpl_segment->axis_seg[1];
    sendZ = orgZ+interpl_segment->axis_seg[2];

    if (isSegDone) {
        finishFlag = true;
        planFlag = false;
        vbg = seg.ve;
    }
    return isSegDone;
}
