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
#include "Comm.h"
#include "StepperClass.h"

/*********************************************************************************************************
** Function name:       init
** Descriptions:        Communication handling initialization
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void CommClass::init(void)
{
    stateMachine = 0;

    usedBufferCount = 0;
    numTestJ = 0;

    tempState = 0;
    tempAxis = 0;
    tempX = 0;
    tempY = 0;
    tempZ = 0;
    tempR1Head = 0;
    tempIsGrab = 0;
    tempStartVel = 0;
    tempEndVel = 0;
    tempMaxVel = 0;

    pinMode(SerGetPin, INPUT);
    selectedCommPort = getSelectedCommPort(SerGetPin);
}

/*********************************************************************************************************
** Function name:       getSelectedCommPort
** Descriptions:        Check which comm port is selected
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
byte CommClass::getSelectedCommPort(byte _SerGetPin)
{
    byte _Ser_num = digitalRead(_SerGetPin) == 0 ? 0 : 1;

    return _Ser_num;
}

/*********************************************************************************************************
** Function name:       process
** Descriptions:        Comm port data processing
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void CommClass::process(void)
{
    if(selectedCommPort == 0) {
        processComm0();
    } else if(selectedCommPort == 1) {
        processComm1();
    }
}

/*********************************************************************************************************
** Function name:       processComm0
** Descriptions:        Comm port data 0 processing
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void CommClass::processComm0(void)
{
    static int gFullCount = 0, gInvalidCount = 0, gTooLongCount = 0;

    if (Serial.available() == 0) {
        return;
    }
    if (stateMachine == 0) {
        while(Serial.available()) {
            if(Serial.read() == 0xa5) {
                stateMachine = 1;
                return;
            }
        }
    } else {
        if (Serial.available() == 41) {
            gFullCount++;
            stateMachine = 0;
            unsigned char dataBuf[41] = {0};

            for(uint32_t counter = 0; counter < 41; counter++) {
                dataBuf[counter] = Serial.read();
            }
            if(dataBuf[40] == 0x5a) {
                if(usedBufferCount >= 0 && usedBufferCount < 6) {
                    for(uint32_t num_j = 0; num_j < 10; num_j++) {
                        *((unsigned char *)(&Parameter_Buf[usedBufferCount][num_j])) = dataBuf[4 * num_j];
                        *((unsigned char *)(&Parameter_Buf[usedBufferCount][num_j]) + 1) = dataBuf[4 * num_j + 1];
                        *((unsigned char *)(&Parameter_Buf[usedBufferCount][num_j]) + 2) = dataBuf[4 * num_j + 2];
                        *((unsigned char *)(&Parameter_Buf[usedBufferCount][num_j]) + 3) = dataBuf[4 * num_j + 3];
                    }
                    usedBufferCount++;
                }
            } else {
                gInvalidCount++;
            }
            Serial1.print(gFullCount);Serial1.print(",");
            Serial1.print(gInvalidCount);Serial1.print(",");
            Serial1.println(gTooLongCount);
        } else if (Serial.available() > 41) {
            stateMachine = 0;
            gFullCount++;
            gTooLongCount++;
            Serial1.print(gFullCount);Serial1.print(",");
            Serial1.print(gInvalidCount);Serial1.print(",");
            Serial1.println(gTooLongCount);
        }
    }
}

typedef struct tagActionLut {
    const char *actionStr;
    const char *printStr;
    int addedVoiceNum;
    int voiceState;
} ActionLut;

static const ActionLut gActionLut[] = {
    {"ahead", "ahead", 0, AHEAD},
    {"back", "back", 0, BACK},
    {"left", "left", 0, LEFT},
    {"right", "right", 0, RIGHT},
    {"up", "up", 0, UP},
    {"down", "down",0, DOWN},
    {"catch", "catch", 0, CATCH},
    {"release", "release", 0, RELEASE},

    {"Ahead", "ahead", 0, AHEAD},
    {"Back", "back", 0, BACK},
    {"Left", "left", 0, LEFT},
    {"Right", "right", 0, RIGHT},
    {"Up", "up", 0, UP},
    {"Down", "down",0, DOWN},
    {"Catch", "catch", 0, CATCH},
    {"Release", "release", 0, RELEASE}
};

static const ActionLut gNumberLut1[] = {
    {"hundred", "hundred", 100, 0}
};

static const ActionLut gNumberLut2[] = {
    {"ten", "ten", 10, 0},
    {"eleven", "eleven", 11, 0},
    {"twelve", "twelve", 12, 0},
    {"thirteen", "thirteen", 13, 0},
    {"fourteen", "fourteen", 14, 0},
    {"fifteen", "fifteen", 15, 0},
    {"sixteen", "sixteen", 16, 0},
    {"seventeen", "seventeen", 17, 0},
    {"eighteen", "eighteen", 18, 0},
    {"nineteen", "nineteen", 19, 0}
};

static const ActionLut gNumberLut3[] = {
    {"twenty", "twenty", 20, 0},
    {"thirty", "thirty", 30, 0},
    {"forty", "forty", 40, 0},
    {"fifty", "fifty", 50, 0},
    {"sixty", "sixty", 60, 0},
    {"seventy", "seventy", 70, 0},
    {"eighty", "eighty", 80, 0},
    {"ninety", "ninety", 90, 0},
};

static const ActionLut gNumberLut4[] = {
    {"one", "one", 1, 0},
    {"two", "two", 2, 0},
    {"three", "three", 3, 0},
    {"four", "four", 4, 0},
    {"five", "five", 5, 0},
    {"six", "six", 6, 0},
    {"seven", "seven", 7, 0},
    {"eight", "eight", 8, 0},
    {"nine", "nine", 9, 0},
};

static const ActionLut *gNumberLut[] = {
    gNumberLut1,
    gNumberLut2,
    gNumberLut3,
    gNumberLut4,
};

static int gNumberLutLength[] = {
    sizeof(gNumberLut1) / sizeof(gNumberLut1[0]),
    sizeof(gNumberLut2) / sizeof(gNumberLut2[0]),
    sizeof(gNumberLut3) / sizeof(gNumberLut3[0]),
    sizeof(gNumberLut4) / sizeof(gNumberLut4[0])
};

void CommClass::processComm1(void)
{
    if (Serial1.available() == 0) {
        return;
    }
    if(stateMachine == 0) {
        while(Serial1.available()) {
            byte dataRead = Serial1.read();
            if(dataRead == 0xa5) {
                stateMachine =1;
                return;
            } else if (dataRead == 0xb5) {
                stateMachine =2;
                return;
            }
        }
    } else if(stateMachine == 1) {
        voiceFlag = false;
        unsigned int num_serial1 = Serial1.available();
        if(num_serial1 >= 41) {
            stateMachine = 0;
            unsigned char dataBuf[41] = {0};
            for(uint32_t counter = 0; counter < 41; counter++) {
                dataBuf[counter] = Serial1.read();
            }
            if(dataBuf[40] == 0x5a) {
                if(usedBufferCount >= 0 && usedBufferCount < 6) {
                    for(uint32_t num_j = 0;num_j < 10;num_j++) {
                        *((unsigned char *)(&Parameter_Buf[usedBufferCount][num_j])) = dataBuf[4 * num_j];
                        *((unsigned char *)(&Parameter_Buf[usedBufferCount][num_j]) + 1) = dataBuf[4 * num_j + 1];
                        *((unsigned char *)(&Parameter_Buf[usedBufferCount][num_j]) + 2) = dataBuf[4 * num_j + 2];
                        *((unsigned char *)(&Parameter_Buf[usedBufferCount][num_j]) + 3) = dataBuf[4 * num_j + 3];
                    }
                    usedBufferCount++;
                }
            } else {
                Serial.println("Invalid header!");
            }
        }
    } else if(stateMachine == 2) {
        tempState = 5;
        voiceFlag = true;
        unsigned int num_serial1 = Serial1.available();
        if(num_serial1 >= 41) {
            stateMachine = 0;
            unsigned char dataBuf[41] = {0};
            for(uint32_t counter = 0; counter < 41; counter++) {
                dataBuf[counter] = Serial1.read();
            }
            if(dataBuf[40] == 0x5b) {
                voiceState = 0;
                voiceNum = 0;

                for (uint32_t i = 0; i < sizeof(gActionLut) / sizeof(gActionLut[0]); i++) {
                    const ActionLut &lut = gActionLut[i];
                    if (strstr((char *)dataBuf, lut.actionStr) != NULL) {
                        Serial.print(lut.printStr);
                        voiceNum += lut.addedVoiceNum;
                        voiceState = lut.voiceState;
                        break;
                    }
                }
                Serial.print(' ');
                for (uint32_t i = 0; i < sizeof(gNumberLut) / sizeof(gNumberLut[0]); i++) {
                    const ActionLut *pNumberLut = gNumberLut[i];
                    for (int j = 0; j < gNumberLutLength[i]; j++) {
                        const ActionLut &lut = pNumberLut[j];
                        if (strstr((char *)dataBuf, lut.actionStr) != NULL) {
                            Serial.print(lut.printStr);
                            voiceNum += lut.addedVoiceNum;
                            break;
                        }
                    }
                }
                if(voiceState == CATCH || voiceState == RELEASE) {
                    usedBufferCount++;
                } else if(voiceState && voiceNum != 0) {
                    usedBufferCount++;
                }
                Serial.print(voiceNum);
            } else {
                Serial.println("Invalid header!");
            }
        }
    }
}

/*********************************************************************************************************
** Function name:       upload
** Descriptions:        Data uploading
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void CommClass::upload(void)
{
    if (selectedCommPort == 0) {
        uploadComm0();
    }
    else if (selectedCommPort == 1) {
        uploadComm1();
    }
}

/*********************************************************************************************************
** Function name:       uploadComm0
** Descriptions:        Data uploading
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void CommClass::uploadComm0()
{
    if (usedBufferCount >= 4) {
        return;
    }
    numTestJ++;
    if(numTestJ > 3) {
        numTestJ = 0;
        currentCoordinate.isGrab = (float)digitalRead(pumpPin);
        Serial.write(0xa5);
        for(unsigned numSend = 0; numSend < 40; numSend++) {
            Serial.write(*((char*)(&currentCoordinate.x) + numSend));
        }
        Serial.write(0x5a);
    }
}

/*********************************************************************************************************
** Function name:       uploadComm1
** Descriptions:        Data uploading
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void CommClass::uploadComm1()
{
    numTestJ++;
    if (numTestJ == 1 || numTestJ == 5) {
        currentCoordinate.x += 1000;

        currentCoordinate.isGrab = (float)digitalRead(pumpPin);
        Serial1.write(0xa5);
        for(uint8_t numSend = 0; numSend < 40; numSend++) {
            Serial1.write(*((char*)(&currentCoordinate.x)+numSend));
        }
        Serial1.write(0x5a);
        currentCoordinate.x -= 1000;
    } else if (numTestJ == 3) {
        // The request
        if (usedBufferCount >= 6) {
            return;
        }
        currentCoordinate.isGrab = (float)digitalRead(pumpPin);
        Serial1.write(0xa5);
        for(unsigned numSend = 0; numSend < 40; numSend++) {
            Serial1.write(*((char*)(&currentCoordinate.x)+numSend));
        }
        Serial1.write(0x5a);
    } else if (numTestJ == 8) {
        numTestJ = 0;
    }
}

/*********************************************************************************************************
** Function name:       getCode
** Descriptions:        Get current communication code
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void CommClass::getCode(void)
{
    if (usedBufferCount <= 0 || usedBufferCount >= 7) {
        return;
    }
    tempState = voiceFlag ? 5 : Parameter_Buf[0][0];
    tempAxis = Parameter_Buf[0][1];

    if (selectedCommPort == 1 && tempState == 2) {
        switch ((int)tempAxis) {
            case 1:
                tempAxis = 3;
            break;
            case 2:
                tempAxis = 4;
            break;
            case 3:
                tempAxis = 1;
            break;
            case 4:
                tempAxis = 2;
            break;
            case 5:
                tempAxis = 6;
            break;
            case 6:
                tempAxis = 5;
            break;
        }
    }
    tempX = Parameter_Buf[0][2];
    tempY = Parameter_Buf[0][3];
    tempZ = Parameter_Buf[0][4];
    tempR1Head = Parameter_Buf[0][5];
    tempIsGrab = Parameter_Buf[0][6];
    tempStartVel = Parameter_Buf[0][7];
    tempEndVel = Parameter_Buf[0][8];
    tempMaxVel = Parameter_Buf[0][9];

    if(usedBufferCount > 1 && usedBufferCount < 7) {
        for(uint32_t DataMove_num = 0; DataMove_num < usedBufferCount - 1; DataMove_num++) {
            for(uint32_t num_j = 0; num_j < 10; num_j++) {
                Parameter_Buf[DataMove_num][num_j] = Parameter_Buf[DataMove_num + 1][num_j];
            }
        }
    }
    usedBufferCount--;
}
