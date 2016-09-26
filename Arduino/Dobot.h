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
#ifndef DOBOT_H
#define DOBOT_H

#include <Arduino.h>
#include "StepperClass.h"
#include "Comm.h"

#define ARM_STRETCH_MIN         130
#define ARM_STRETCH_MAX         320
#define ARM_HEIGHT_MIN          -35
#define ARM_HEIGHT_MAX          135

#define ARM_A1_MIN              -5
#define ARM_A1_MAX              85
#define ARM_A2_MIN              -10
#define ARM_A2_MAX              95

#define ARM_A12_MIN             5
#define ARM_A12_MAX             125
#define BASE_ROTATION_MIN       -180
#define BASE_ROTATION_MAX       180
#define HAND_ROTATION_MIN       -90
#define HAND_ROTATION_MAX       90

#define STEP_B                 1
#define STEP_H                 2
#define STEP_V                 3
#define STEP_P                 4

enum
{
    IDEL,
    AP_DOWN,
    AN_DOWN,
    BP_DOWN,
    BN_DOWN,
    CP_DOWN,
    CN_DOWN,
    DP_DOWN,
    DN_DOWN,
    Grip_Catch,
    Grip_Release,
    EP_DOWN,
    EN_DOWN,
};

enum
{
  JMP,
  JOINT,
  LINE,

};

#ifndef PI
#define PI                      3.1416
#endif

#define INTERP_CYC              (float)0.020
#define INTERP_RECI             (float)50

#define LINE_MIN_DIST           0.0001f

#define LDEGREE                 0.1f
#define INP_MOTION_END          0x50
#define AXIS_SUM                3

#define SMOOTHING_FRAMES        10

class Dobot
{
public:
    Dobot(void);
    void mouseControl(float dx,float dy,float dz,float r,float isGrab);
    void singleAxisControl(int state);
    void visionControl(float x, float y, float z,  float r,float isGrab,float  gripperAngle, float delayTime,int type);
    void write(float vs, float ve, float vm, float dx, float dy, float dz,float stopflag,float isLaserOn,float state);
    void playback(float b, float a1, float a2, float p,float isGrab,float gripperAngle, float delayTime,int type);
    void singleXYZControl(int state);
    void motionControl(float x,float y,float z,float r,float isGrab);
    void easyControl(float a1, float a2, float b, float p);
    void getCurPositionXYZR(float b, float a1, float a2,  float p);
    void getCurrentCoord(void);
    void voiceControl(float dx,float dy,float dz,float dp);
public:
    bool finishFlag;
    bool planFlag;

    float coordAxisB;
    float coordAxisH;
    float coordAxisV;
    float coordAxisP;
    float coordAxisG;

    int step1;
    int step2;
    int step3;
    int step4;

    float accWrite;

    StepperClass stepper;

    int length;

    float singleVelMaxB;
    float singleVelMaxH;
    float singleVelMaxV;
    float singleVelMaxP;

    float singleAccB;
    float singleAccH;
    float singleAccV;
    float singleAccP;

    float singleVelMaxX;
    float singleVelMaxY;
    float singleVelMaxZ;
    float singleVelMaxR;

    float singleAccX;
    float singleAccY;
    float singleAccZ;
    float singleAccR;

    float singleVelRatio;

    float linkBlockH;
    float heightMin;
private:
    void movjABS(float a1, float a2, float b, float p);

    void setStepSpeed(char stepNum, float stepSpeed);
    void setStepDir(int dir1, int dir2, int dir3);

    void getDistPositionXYZR(float b, float a1, float a2, float p);

    void movjACC(float x, float y, float z,  float r);

    void movjHVBP(float h, float v,float b, float p);
    void movjXYZEven(float x, float y,float z, float r);

    void inverse(float x, float y, float z,  float r);

    void movjINC(float db,float da1,float da2,float dp);
    void movj(float b, float a1, float a2,  float p);
    void movjXYZ(float x, float y, float z,  float r);

    void movXYZ(float dx, float dy, float dz,  float dr);

    void movlINC(float dx, float dy, float dz,  float dr);
    void movl(float b, float a1, float a2,  float p);
    void movlXYZ(float x, float y, float z,  float r);

    void jump(float b, float a1, float a2,  float p);
    void jumpXYZ(float x, float y, float z,  float r);

    void planSpeed(float vs, float ve, float vm, float dx, float dy,float dz,float stopflag,float isLaserOn,float state);

    void getSpeedJoint(void);
    void getSpeedHVBP(void);
    void getSpeedLine(void);
    void writePulse(float b, float h, float v, float p);
    void writeGripperPulse(float g);

    void single(int state);
    void singlePos(void);
    void singleNeg(void);
    void singleStop(void);
    void singleXY(int state);
    void singleXYStop(void);

    void getNextPositionXYZR(float b, float a1, float a2, float p, CurrentCoordinateType *coord);
public:
    float stepSpdL;
    float stepAccL;

    float stepSpdB;
    float stepSpdH;
    float stepSpdV;
    float stepSpdP;

    float stepAccB;
    float stepAccH;
    float stepAccV;
    float stepAccP;

    float velRatio;
    float accRatio;

    float pauseTime;
    float height;

    CurrentCoordinateType coorGet;

    float handHistory[10][3];
    float handPosition[3];

    bool limitFlag;
private:
    bool firstLimitFlag;
    float samplingPeriod;

    float arm1Len;
    float arm2Len;
    float baseH;
    float baseV;

    float linkBlockV;

    float stepAngle;

    float subdiv1;
    float subdiv2;
    float subdiv3;
    float gearRatio1;
    float gearRatio2;
    float gearRatio3;

    float periodTime;

    float stepDir1;
    float stepDir2;
    float stepDir3;

    float coordX;
    float coordY;
    float coordZ;
    float coordR;

    float sendSpeedBSum;
    float sendSpeedHSum;
    float sendSpeedVSum;
    float sendSpeedPSum;

    int sendPulseSumB;
    int sendPulseSumH;
    int sendPulseSumV;
    int sendPulseSumP;

    float remainPulseB;
    float remainPulseH;
    float remainPulseV;
    float remainPulseP;

    struct Axis
    {
        float s;
        float v;
        float t1;
        float t2;
        float T;
        float a;
        bool finishFlag;
    };

    Axis axisB;
    Axis axisH;
    Axis axisV;
    Axis axisP;
    Axis line;

    float targetB;
    float targetH;
    float targetV;
    float targetP;

    int dirB;
    int dirH;
    int dirV;
    int dirP;

    float stepLastB;
    float stepLastH;
    float stepLastV;
    float stepLastP;
    float stepLastL;

    float speedH;
    float speedV;
    float speedB;
    float speedP;
    float sendServoP;

    float distX;
    float distY;
    float distZ;
    float distR;

    bool transFlag;
    bool jumPlanFlag;
    bool downFlag;

    bool pauseFlag;
    bool catchFlag;
    int pauseCount;

    float T;
    int periodCount;
    float T1;
    float T2;
    float t0;

    float orgX;
    float orgY;
    float orgZ;
    float orgR;

    float dirX;
    float dirY;
    float dirZ;

    float speedLine;

    float solvedB;
    float solvedH;
    float solvedV;
    float solvedP;

    float singleVel;
    float singleVelMax;
    float singleAcc;

    int lastState;

    float sendH;
    float sendV;
    float sendB;
    float sendP;

    float sendX;
    float sendY;
    float sendZ;
    float sendR;

    typedef struct
    {
        float state;
        float axisE;
        float x;
        float y;
        float z;
        float rHead;
        float isGrab;
        float startVel;
        float endVel;
        float maxVel;
    } PackageStruct;

    typedef struct{
        int motion_stat;
        int motion_stat_E;

        long term_cmd;
        long term_run;
        long term_rem;

        int term_add;
        int term_unfm;
        int term_dec;

        float AT;
        float time_delta;
        float term_length;
        float term_vel;
        float vel;
        float vDelta;
    }typ_interp_time;

    typedef struct
    {
        PackageStruct gCode;
        float dist;
        float accelat;
        float axis_vel[3];
        float dist_out[3];

        typ_interp_time interp_time;
        float axis_seg[3];

        float vb;
        float vm;
        float ve;
        float vc;
        float vd;
        float va;
    }typ_interp_segment;

    typ_interp_segment seg;

    float absX(float a);

    void line_interp_plan(typ_interp_segment * interpl_segment);
    int linear_vel_ctrl(typ_interp_time * interp_time);
    int line_interp_deal(typ_interp_segment * interpl_segment);
};
#endif
