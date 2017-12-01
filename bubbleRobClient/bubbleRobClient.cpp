// Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
//
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

// Make sure to have the server side running in V-REP!
// Start the server from a child script with following command:
// simExtRemoteApiStart(portNumber) -- starts a remote API server service on the specified port

#include <stdio.h>
#include <stdlib.h>

extern "C" {
    #include "extApi.h"
}

int main(int argc,char* argv[])
{
    int horizontalMotor;
    int verticalMotor;
    int visionSensorHandler;


        printf("Laczenie z serwerem V-REP Remote Api...\n");
    int clientID=simxStart("127.0.0.1",20000,true,true,2000,5);

        if (clientID == -1)
        {
                printf("Blad laczenia\n");
                return 0;
        }

        int ret1 = simxGetObjectHandle(clientID, "jh", &horizontalMotor, simx_opmode_blocking);
        int ret2 = simxGetObjectHandle(clientID, "jv", &verticalMotor, simx_opmode_blocking);
        int ret3 = simxGetObjectHandle(clientID, "vision_sensor", &visionSensorHandler, simx_opmode_blocking);

        if (ret1 != simx_return_ok || ret2 != simx_return_ok || ret3 != simx_return_ok)
        {
                printf("Blad komunikacji (1)\n");
                printf("ret1=%d; ret2=%d; ret3=%d\n", ret1, ret2, ret3);
                simxFinish(clientID);
                return 0;
        }

        printf("Uchwyt remoteApiControlledBubbleRobLeftMotor = %d\n", horizontalMotor);
        printf("Uchwyt remoteApiControlledBubbleRobRightMotor = %d\n", verticalMotor);
        printf("Uchwyt remoteApiControlledBubbleRobSensingNose = %d\n", visionSensorHandler);

        //

        int driveBackStartTime=-99000;
        float motorSpeeds[2];

        motorSpeeds[0] = -3.1415f*0;
        motorSpeeds[1] = -3.1415f*0;
        simxSetJointTargetVelocity(clientID, horizontalMotor, motorSpeeds[0], simx_opmode_oneshot);
        simxSetJointTargetVelocity(clientID, verticalMotor, motorSpeeds[1], simx_opmode_oneshot);
        extApi_sleepMs(2000);

    while (simxGetConnectionId(clientID)!=-1)
    {
        printf(".");
        fflush(stdout);
        simxFloat** auxValues = (simxFloat**)simxCreateBuffer(4*sizeof(simxFloat*));
        simxInt* auxValuesCount;
        for (int i =0; i < 4; i++){
            auxValues[i] = (simxFloat*)simxCreateBuffer(35*sizeof(simxFloat));
        }
        simxUChar sensorTrigger=0;

        if (simxReadVisionSensor(clientID,visionSensorHandler, &sensorTrigger, auxValues, &auxValuesCount,simx_opmode_streaming)==simx_return_ok)
        {
            // We succeeded at reading the vision sensor
            if(auxValues[0][8] >= 0.85) {
                    printf("*");
                    fflush(stdout);
                    double ox = auxValues[0][19];
                    double oy = auxValues[0][20];
                    int simulationTime=simxGetLastCmdTime(clientID);
                    if (simulationTime-driveBackStartTime<3000)
                    { // driving backwards while slightly turning:
                            motorSpeeds[0]=-1*( 0.5f - ox);
                            motorSpeeds[1]=-1*( 0.5f - oy);
                    }
                    else
                    { // going forward:
                            motorSpeeds[0]= 5.5 * ( 0.5f - ox);
                            motorSpeeds[1]= 3 * ( 0.5f - oy);
                            if (sensorTrigger)
                                    driveBackStartTime=simulationTime;
                    }
                simxSetJointTargetVelocity(clientID,horizontalMotor,motorSpeeds[0],simx_opmode_oneshot);
                simxSetJointTargetVelocity(clientID,verticalMotor,motorSpeeds[1],simx_opmode_oneshot);
            }
            else
            {
                motorSpeeds[0]=0;
                motorSpeeds[1]=0;
                simxSetJointTargetVelocity(clientID,horizontalMotor,motorSpeeds[0],simx_opmode_oneshot);
                simxSetJointTargetVelocity(clientID,verticalMotor,motorSpeeds[1],simx_opmode_oneshot);
            }
        }
        extApi_sleepMs(5);
    }
    simxFinish(clientID);
}

