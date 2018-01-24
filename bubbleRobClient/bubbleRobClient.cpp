#include <stdio.h>
#include <stdlib.h>
#include <math.h>

extern "C" {
    #include "extApi.h"
}

int main(int argc,char* argv[])
{
    int bubbleRobLeftMotor;
    int bubbleRobRightMotor;
    int bubbleRobFrontSensor;

    printf("Connecting to server V-REP Remote Api...\n");
    int clientID=simxStart("127.0.0.1",20000,true,true,2000,5);

    if (clientID == -1)
    {
            printf("Blad laczenia\n");
            return 0;
    }

    int ret1 = simxGetObjectHandle(clientID, "bubbleRob_leftMotor", &bubbleRobLeftMotor, simx_opmode_blocking);
    int ret2 = simxGetObjectHandle(clientID, "bubbleRob_rightMotor", &bubbleRobRightMotor, simx_opmode_blocking);
    int ret3 = simxGetObjectHandle(clientID, "bubbleRob_frontSensor", &bubbleRobFrontSensor, simx_opmode_oneshot_wait);

    if (ret1 != simx_return_ok || ret2 != simx_return_ok || ret3 != simx_return_ok)
    {
            printf("Communication error (1)\n");
            printf("ret1=%d; ret2=%d; ret3=%d\n", ret1, ret2, ret3);
            simxFinish(clientID);
            return 0;
    }
   
    printf("Object bubbleRobLeftMotor handler= %d\n", bubbleRobLeftMotor);
    printf("Object bubbleRobRightMotor handler= %d\n", bubbleRobRightMotor);
    printf("Object bubbleRobFrontSensor handler= %d\n", bubbleRobFrontSensor);
    
    printf("Object handlers received successfully.\n");

    float motorSpeeds[2];

    motorSpeeds[0] = 3.1415f*0;
    motorSpeeds[1] = 3.1415f*0;
    
    simxSetJointTargetVelocity(clientID, bubbleRobLeftMotor, motorSpeeds[0], simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, bubbleRobRightMotor, motorSpeeds[1], simx_opmode_oneshot);
    extApi_sleepMs(2000);
    
    printf("Velocity set to L=%f, R=%f.\n", motorSpeeds[0], motorSpeeds[1]);
	
	simxUChar detectionState = 0;
	simxFloat detectedPoint[3];
	
	while (simxGetConnectionId(clientID)!=-1)
	{
		int result = simxReadProximitySensor(clientID, bubbleRobFrontSensor, &detectionState, detectedPoint, NULL, NULL, simx_opmode_oneshot);
	
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
				
				float distanceFromObject = *(detectedPoint+2); 	//z coordinate - distance
				printf("front sensor distance = %.2f \n", distanceFromObject);
			}
			else {
			printf("no objects detected!\n");
				motorSpeeds[0] = 3.1415f*1;
				motorSpeeds[1] = 3.1415f*1;
			}

			simxSetJointTargetVelocity(clientID, bubbleRobLeftMotor, motorSpeeds[0], simx_opmode_oneshot);
			simxSetJointTargetVelocity(clientID, bubbleRobRightMotor, motorSpeeds[1], simx_opmode_oneshot);

			extApi_sleepMs(5);
		}
	}
    simxFinish(clientID);
}


//sterowanie =wzmocnienieP*uchyb + stalaD*((uchyb-uchyb_pop)/czas) + stalaI*calka(uchyb);
/*int dt = 1; //co ile pobiera się próbke
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
*/
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
