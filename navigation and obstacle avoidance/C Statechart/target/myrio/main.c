/** \file main.c
 *
 * Top-level application for navigating the iRobot Create using
 * a myRIO microcontroller.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include "MyRio.h"
#include "Accelerometer.h"
#include "UART.h"
#include "irobot.h"
#include "irobotNavigationStatechart.h"
#include "irobotSensorTypes.h"

/// sensor roll
void rroll(
	const irobotSensorGroup6_t * const pSensors,///< [in] iRobot sensors
	const irobotUARTPort_t port					///< [in] UART port
);

/// System clock in ms
/// \returns System clock in ms
uint64_t getTimeInMs(void);

/// Delay
void delayMs(
	const uint64_t msDelay		///< [in] Length of delay, in ms
);

/// Wait until the system clock passes the next integer multiple of a period.
/// Use to create periodic loops. This function will never delay more than
///	msMultiple, but may delay less.
void waitUntilNextMsMultiple(
	const uint64_t msMultiple	///< Multiple clock must pass before unblocking, in ms
);

const int32_t driveDistance = 200;		// distance to drive, in mm
const int32_t turnAngle = 90;			// angle to turn, in mm
const double alpha = 0.2;				// accelerometer filter coefficient

int main(int argc, char **argv)
{
	// Hardware peripherals
	MyRio_Accl				accelDevice;
	irobotUARTPort_t 		port = UART1;			///< UART port

	// sensor inputs
	irobotSensorGroup6_t	sensors;				///< irobot sensors
	int32_t					netDistance = 0;		///< net distance the robot has traveled, in mm
	int32_t					netAngle = 0;			///< net angle through which the robot has turned, in deg
	accelerometer_t			accelValue = {0,0,0};	///< accelerometer, in g
	static accelerometer_t	accelPrevValue ={0,0,0};///< previous accelerometer value

	// actuator outputs
	int16_t					leftWheelSpeed = 0;		///< speed of the left wheel, in mm/s
	int16_t					rightWheelSpeed = 0;	///< speed of the right wheel, in mm/s

    NiFpga_Status 			status;

    status = MyRio_Open();
    if (MyRio_IsNotSuccess(status)){
    	MyRio_PrintStatus(status);
        return status;
    }

	// Specify the registers that correspond to the accelerometer channen that needs to be accessed.
	accelDevice.xval = ACCXVAL;
	accelDevice.yval = ACCYVAL;
	accelDevice.zval = ACCZVAL;
	accelDevice.scale_wght = ACCSCALEWGHT;
	Accel_Scaling(&accelDevice);

	// initialize iRobot */
	NiFpga_IfIsNotError(status, irobotOpen(port));

	// Read inputs, execute statechart, generate outputs, print debug information */
	while(!NiFpga_IsError(status) && !sensors.buttons.advance){
		// Read iRobot sensors
		NiFpga_IfIsNotError(status, irobotSensorPollSensorGroup6(port, &sensors));
		if(NiFpga_IsNotError(status)){
			// accumulate distance and angle
			netDistance += sensors.distance;
			netAngle += sensors.angle;
		}

		// Read and filter accelerometer
		if(NiFpga_IsNotError(status)){
			accelValue.x = Accel_ReadX(&accelDevice);
			accelValue.y = Accel_ReadY(&accelDevice);
			accelValue.z = Accel_ReadZ(&accelDevice);
		}
		if(NiFpga_IsNotError(status)){
			accelValue.x = alpha * accelValue.x + (1 - alpha) * accelPrevValue.x;
			accelValue.y = alpha * accelValue.y + (1 - alpha) * accelPrevValue.y;
			accelValue.z = alpha * accelValue.z + (1 - alpha) * accelPrevValue.z;
			accelPrevValue = accelValue;
		}

		// Execute statechart
		irobotNavigationStatechart(
			netDistance,
			netAngle,
			sensors,
			accelValue,
			false,
			&leftWheelSpeed,
			&rightWheelSpeed
		);

		// Produce outputs
		NiFpga_IfIsNotError(status, irobotDriveDirect(port, leftWheelSpeed, rightWheelSpeed));

		// print debug information
		printf("\n\nx=%+.2f y=%+.2f z=%+.2f\nLWheel=%+3d RWheel=%+3d\n",
				accelValue.x,
				accelValue.y,
				accelValue.z,
				leftWheelSpeed,
				rightWheelSpeed);

		// try uncommenting this line */
		// rroll(&sensors, port); */

		// Loop timing
		waitUntilNextMsMultiple(60);
	}

	// even if an error has occurred, close the UART port
	NiFpga_MergeStatus(&status, irobotClose(port));

    MyRio_Close();

    MyRio_PrintStatus(status);

    return status;
}

uint64_t getTimeInMs(void){
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000;
}

void delayMs(const uint64_t msDelay){
	usleep(msDelay * 1000);
}

void waitUntilNextMsMultiple(const uint64_t msMultiple){
	const uint64_t msCounter = getTimeInMs() % msMultiple;
	if(msCounter > 0){
		delayMs(msMultiple - msCounter);
	}
}

void rroll(const irobotSensorGroup6_t * const pSensors, const irobotUARTPort_t port){
	static uint8_t bInitialized = 0;

	if(!bInitialized){
		bInitialized = 1;

		// initialize rr
		const uint8_t rrInit[111] = {140,0,54,72,8,74,8,77,8,74,8,81,
			32,81,32,79,32,72,8,74,8,77,8,74,8,79,32,79,32,77,32,72,8,
			74,8,77,8,74,8,77,32,79,16,76,24,74,8,72,32,72,16,79,32,
			77,64,72,8,74,8,77,8,74,8,81,32,81,32,79,32,72,8,74,8,77,
			8,74,8,84,32,76,16,77,24,76,8,74,16,72,8,74,8,77,8,
			74,8,77,32,79,16,76,24,74,8,72,32,72,16,79,32,77,64};
		irobotUARTWriteRaw(port, rrInit, 111);
	}

	if(!pSensors->songPlaying){
		const uint8_t rr[2] = {141,0};
		irobotUARTWriteRaw(port, rr, 2);
	}
}
