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
	
    int motorFrontLeft;
    int motorFrontRight;
    
    int frontLeftSensor;
    int frontSensor;
    int frontRightSensor;


    printf("Laczenie z serwerem V-REP Remote Api...\n");
    int clientID=simxStart("127.0.0.1",20000,true,true,2000,5);

    if (clientID == -1)
    {
            printf("Blad laczenia\n");
            return 0;
    }

    int ret1 = simxGetObjectHandle(clientID, "motorFrontLeft", &motorFrontLeft, simx_opmode_blocking);
    int ret2 = simxGetObjectHandle(clientID, "motorFrontRight", &motorFrontRight, simx_opmode_blocking);
    
    int ret3 = simxGetObjectHandle(clientID, "frontProxSensor", &frontSensor, simx_opmode_oneshot_wait);
    int ret4 = simxGetObjectHandle(clientID, "frontLeftSensor", &frontLeftSensor, simx_opmode_oneshot_wait);
    int ret5 = simxGetObjectHandle(clientID, "frontRightSensor", &frontRightSensor, simx_opmode_oneshot_wait);

    if (ret1 != simx_return_ok || ret2 != simx_return_ok || ret3 != simx_return_ok || ret4 != simx_return_ok|| ret5 != simx_return_ok)
    {
            printf("Blad komunikacji (1)\n");
            printf("ret1=%d; ret2=%d; ret3=%d\n", ret1, ret2, ret3, ret4, ret5);
            simxFinish(clientID);
            return 0;
    }

    printf("Uchwyt motorFrontLeft = %d\n", motorFrontLeft);
    printf("Uchwyt motorFrontRight = %d\n", motorFrontRight);
    printf("Uchwyt frontSensor = %d\n", frontSensor);
    printf("Uchwyt frontLeftSensor = %d\n", frontLeftSensor);
    printf("Uchwyt frontRightSensor = %d\n", frontRightSensor);
    
  	//sterowanie =wzmocnienieP*uchyb + stalaD*((uchyb-uchyb_pop)/czas) + stalaI*calka(uchyb);
    int dt = 1; //co ile pobiera się próbke

	float epX = 0; //uchyb poprzedni
	float enX = 0; //uchyb następny

	float Ux = 0; //sygnał sterujący
	float Cx = 0; //część całkująca

	float epY = 0; //uchyb poprzedni
	float enY = 0; //uchyb następny

	float Uy = 0; //sygnał sterujący
	float Cy = 0; //część całkująca

	float Kp = 1; //wzmocnienie
	float Ti = 1; //stała całkowania
	float Td = 0; //stała różniczkowania
	
	printf("Zmienne PID zainicjalizowane.\n");

    float motorSpeeds[2];

    motorSpeeds[0] = 3.1415f*0;
    motorSpeeds[1] = 3.1415f*0;
    simxSetJointTargetVelocity(clientID, motorFrontLeft, motorSpeeds[0], simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, motorFrontRight, motorSpeeds[1], simx_opmode_oneshot);
    extApi_sleepMs(2000);
    
    printf("Prędkości ustawione na L=%f, R=%f.\n", motorSpeeds[0], motorSpeeds[1]);
    
    //simxFloat* detectedPoint = (simxFloat*)simxCreateBuffer(4*sizeof(simxFloat*));
	//simxInt* auxValuesCount = (simxInt**)simxCreateBuffer(4*sizeof(simxInt*));
	
	simxUChar detectionState = NULL;
	simxFloat detectedPoint[3];
	
	
	while (simxGetConnectionId(clientID)!=-1)
	{
		
		int result = simxReadProximitySensor(clientID, frontSensor, &detectionState, detectedPoint, NULL, NULL, simx_opmode_oneshot);
	
		if (result != simx_return_ok){
			printf("Error while reading from proximity sensor. Error nr: %d\n", result);
			extApi_sleepMs(2000);
		}
	
		if (result == simx_return_ok)
		{
			fflush(stdout);

			if (detectionState == 1) {
				printf("WYKRYTO OBIEKT!\n");

				motorSpeeds[0] = 3.1415f*0;
				motorSpeeds[1] = 3.1415f*0;
				
				float frontSensorDistance = *(detectedPoint+2); 	//z coordinate
				printf("front sensor distance = %.2f \n", frontSensorDistance);
			}
			else {
			printf("no objects detected!\n");
				motorSpeeds[0] = 3.1415f*2;
				motorSpeeds[1] = 3.1415f*2;
			}

			simxSetJointTargetVelocity(clientID, motorFrontLeft, motorSpeeds[0], simx_opmode_oneshot);
			simxSetJointTargetVelocity(clientID, motorFrontRight, motorSpeeds[1], simx_opmode_oneshot);


			/*printf("Xball: %f Yball: %f \n", ox, oy);

			epX = -(0.5-auxValues[0][19]);
			Cx += ((epX + enX)/2)*dt;
			motorSpeeds[0] = Kp*(epX + (1/Ti)*Cx + Td*(enX - epX)/dt);
			epX = enX;
			printf("Ux = %f \n", Ux);

			epY = -(0.5-auxValues[0][20]);
			Cy += ((epY + enY)/2)*dt;
			motorSpeeds[1] = Kp*(epY + (1/Ti)*Cy + Td*(enY - epY)/dt);
			epY = enY;
			printf("Uy = %f \n", Uy);
			*/

			//simxSetJointTargetVelocity(clientID,horizontalMotor, motorSpeeds[0], simx_opmode_oneshot);
			//simxSetJointTargetVelocity(clientID,verticalMotor, motorSpeeds[1], simx_opmode_oneshot);

			extApi_sleepMs(5);
		}
	}
    simxFinish(clientID);
}

