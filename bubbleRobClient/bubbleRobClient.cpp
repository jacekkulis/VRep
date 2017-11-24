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
    int leftMotorHandle;
    int rightMotorHandle;


	printf("Laczenie z serwerem V-REP Remote Api...\n");
    int clientID=simxStart("127.0.0.1",20000,true,true,2000,5);

	if (clientID == -1)
	{
		printf("Blad laczenia\n");
		return 0;
	}
	
	int ret1 = simxGetObjectHandle(clientID, "motorBackLeft", &leftMotorHandle, simx_opmode_blocking);
	int ret2 = simxGetObjectHandle(clientID, "motorBackRight", &rightMotorHandle, simx_opmode_blocking);

        if (ret1 != simx_return_ok || ret2 != simx_return_ok)
	{
		printf("Blad komunikacji (1)\n");
                printf("ret1=%d; ret2=%d;", ret1, ret2);
		simxFinish(clientID);
		return 0;
	}
	
	printf("Uchwyt remoteApiControlledBubbleRobLeftMotor = %d\n", leftMotorHandle);
	printf("Uchwyt remoteApiControlledBubbleRobRightMotor = %d\n", rightMotorHandle);
	 

	int driveBackStartTime=-99000;
        float motorSpeeds[2];

	motorSpeeds[0] = -3.1415f*0.5f;
	motorSpeeds[1] = -3.1415f*-0.5f;
	simxSetJointTargetVelocity(clientID, leftMotorHandle, motorSpeeds[0], simx_opmode_oneshot);
	simxSetJointTargetVelocity(clientID, rightMotorHandle, motorSpeeds[1], simx_opmode_oneshot);
	extApi_sleepMs(5000);


    while (simxGetConnectionId(clientID)!=-1)
    {
        printf(".");
        fflush(stdout);

		
		//
		
       /* simxUChar sensorTrigger=0;
        if (simxReadProximitySensor(clientID,sensorHandle,&sensorTrigger,NULL,NULL,NULL,simx_opmode_streaming)==simx_return_ok)
        { // We succeeded at reading the proximity sensor
            int simulationTime=simxGetLastCmdTime(clientID);
            if (simulationTime-driveBackStartTime<3000)
            { // driving backwards while slightly turning:
                motorSpeeds[0]=-3.1415f*0.5f;
                motorSpeeds[1]=-3.1415f*0.25f;
            }
            else
            { // going forward:
                motorSpeeds[0]=3.1415f;
                motorSpeeds[1]=3.1415f;
                if (sensorTrigger)
                    driveBackStartTime=simulationTime; // We detected something, and start the backward mode
            }
            simxSetJointTargetVelocity(clientID,leftMotorHandle,motorSpeeds[0],simx_opmode_oneshot);            
            simxSetJointTargetVelocity(clientID,rightMotorHandle,motorSpeeds[1],simx_opmode_oneshot);           
        }
        extApi_sleepMs(5)*/;
    }
    simxFinish(clientID);
}
