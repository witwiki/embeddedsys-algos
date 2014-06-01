/** \file irobotNavigationStatechart.h
 *
 * Cross-platform Statechart for navigating the iRobot Create,
 * either on myRIO or in desktop simulation.
 */

#ifndef IROBOTNAVIGATIONSTATECHART_H_
#define IROBOTNAVIGATIONSTATECHART_H_

#define _USE_MATH_DEFINES
#include "irobotSensorTypes.h"

/// accelerometer values
typedef struct{
	double	x;							///< x axis, in g
	double	y;							///< y axis, in g
	double	z;							///< z axis, in g
} accelerometer_t;

/// Architecture-independent C Statechart.
void irobotNavigationStatechart(
	const int32_t 				netDistance,		///< [in] net distance, in mm
	const int32_t 				netAngle,			///< [in] net angle, in deg
	const irobotSensorGroup6_t	sensors,			///< [in] sensor stream size, in bytes
	const accelerometer_t		accelAxes,			///< [in] accelerometer, in g
	const bool					isSimulator,		///< [in] executed by a simulator
	int16_t * const 			pRightWheelSpeed,	///< [out] right wheel speed, in mm/s
	int16_t * const 			pLeftWheelSpeed		///< [out] left wheel speed, in mm/s
);

#endif // IROBOTNAVIGATIONSTATECHART_H_
