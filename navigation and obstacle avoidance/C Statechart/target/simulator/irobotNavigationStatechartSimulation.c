#include "irobotNavigationStatechartSimulation.h"
#include "irobotNavigationStatechart.h"
#include "irobotSensorStream.h"
#include "irobotSensorTypes.h"
#include <stdint.h>
#include <stdio.h>

int32_t LIBSTATECHARTEXAMPLE_EXP irobotNavigationStatechartSimulation(
	const int32_t 			netDistance,
	const int32_t 			netAngle,
	const uint8_t * const 	sensorStream,
	const int32_t			sensorStreamSize,
	const double * const	accelAxes,
	const int32_t			accelAxesSize,
	int16_t * const 		pRightWheelSpeed,
	int16_t * const 		pLeftWheelSpeed
){
	// construct sensors from a simulated sensor stream. Copy into xqueue structure then parse
	xqueue_t 				sensorStreamQueue;
	uint8_t 				sensorStreamQueueBuffer[SENSOR_SIZE_UPPER_BOUND];
	irobotSensorGroup6_t	sensors;
	accelerometer_t			accel;
	bool					packetFound = false;

	if (!sensorStream || !pRightWheelSpeed || !pLeftWheelSpeed || accelAxesSize != 3)
		return 1;	//mgArgErr

	// Verify correct sensor stream packet size
	if(sensorStreamSize == SENSOR_GROUP6_SIZE + 4){
		xqueue_init(&sensorStreamQueue, sensorStreamQueueBuffer, SENSOR_SIZE_UPPER_BOUND);
		xqueue_push_buffer(&sensorStreamQueue, sensorStream, sensorStreamSize);

		if(irobotSensorStreamProcessAll(&sensorStreamQueue, &sensors, &packetFound) >= 0){
			if(packetFound){
				accel.x = accelAxes[0];
				accel.y = accelAxes[1];
				accel.z = accelAxes[2];

				// Execute statechart
				irobotNavigationStatechart(netDistance,
										   netAngle,
										   sensors,
										   accel,
										   true,
										   pRightWheelSpeed,
										   pLeftWheelSpeed);

				return ERROR_SUCCESS;
			}
		}
	}
	else{
		fprintf(stderr,
				"irobotNavigationSensorStream() expected sensor packet size %d, received size %d.\n",
				SENSOR_GROUP6_SIZE + 4,
				sensorStreamSize);
	}

	return ERROR_INVALID_PARAMETER;
}
