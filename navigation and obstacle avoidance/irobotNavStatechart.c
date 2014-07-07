/*
 *	irobotNavStatechart.c
 *
 *	@witwiki #02062014
 *	Finite State Machine for the iRobot Create.
 *	As part of the CyberPhysical Systems program
 *
 */


#include "irobotNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>

// Program States
typedef enum{
	INITIAL = 0,						// Initial state
	PAUSE_WAIT_BUTTON_RELEASE,			// Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,			// Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE,		// Paused; pause button pressed down, wait until released before returning to previous state
	DRIVE,								// Drive straight
	AVOID,								// Avoid an obstacle
	REORIENT							// Reorient after obstacle avoidance
} robotState_t;

#define DEG_PER_RAD			(180.0 / M_PI)		// degrees per radian
#define RAD_PER_DEG			(M_PI / 180.0)		// radians per degree

// state data
static const int32_t driveSpeed = 200;			// normal drive speed, in mm/s
static const int32_t reorientSpeed = 75;		// reorient speed, in mm/s
static const int32_t avoidDistance = 250;		// distance to travel in avoidance algorithm before reorienting
static const int32_t reorientTolerance = 2;		// tolerance for reorienting robot, in deg

typedef enum{
	LEFT,
	RIGHT
} obstacleDirection_t;							// Direction of an encountered obstacle

void irobotNavigationStatechart(
	const int32_t 				netDistance,
	const int32_t 				netAngle,
	const irobotSensorGroup6_t	sensors,
	const accelerometer_t		accelAxes,
	const bool					isSimulator,
	int16_t * const 			pRightWheelSpeed,
	int16_t * const 			pLeftWheelSpeed
){
	// local state
	static robotState_t 		state = INITIAL;				// current program state
	static robotState_t			unpausedState = DRIVE;			// state history for pause region
	static obstacleDirection_t	obstacleDirection = LEFT;		// direction of an obstacle to avoid
	static int32_t				distanceAtManeuverStart = 0;	// distance robot had travelled when a maneuver begins, in mm
	static int32_t				angleAtManeuverStart = 0;		// angle through which the robot had turned when a maneuver begins, in deg

	// outputs
	int16_t						leftWheelSpeed = 0;				// speed of the left wheel, in mm/s
	int16_t						rightWheelSpeed = 0;			// speed of the right wheel, in mm/s

	/******************************************************/
	// state data - process inputs                       
	/******************************************************/

	/******************************************************/
	// state transition - pause region (highest priority)
	/******************************************************/
	if(   state == INITIAL
	   || state == PAUSE_WAIT_BUTTON_RELEASE
	   || state == UNPAUSE_WAIT_BUTTON_PRESS
	   || state == UNPAUSE_WAIT_BUTTON_RELEASE
	   || sensors.buttons.play				// pause button
	){
		switch(state){
		case INITIAL:
			// set state data that may change between simulation and real-world
			if(isSimulator){
			}
			else{
			}
			state = UNPAUSE_WAIT_BUTTON_PRESS; // place into pause state
			break;
		case PAUSE_WAIT_BUTTON_RELEASE:
			// remain in this state until released before detecting next press
			if(!sensors.buttons.play){
				state = UNPAUSE_WAIT_BUTTON_PRESS;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_RELEASE:
			// user pressed 'pause' button to return to previous state
			if(!sensors.buttons.play){
				state = unpausedState;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_PRESS:
			// remain in this state until user presses 'pause' button
			if(sensors.buttons.play){
				state = UNPAUSE_WAIT_BUTTON_RELEASE;
			}
			break;
		default:
			// must be in run region, and pause button has been pressed
			unpausedState = state;
			state = PAUSE_WAIT_BUTTON_RELEASE;
			break;
		}
	}
	/////////////////////////////////////////
	// state transition - obstacle region  //
	/////////////////////////////////////////
	else if(   sensors.bumps_wheelDrops.bumpLeft
			|| sensors.bumps_wheelDrops.bumpRight
			|| sensors.bumps_wheelDrops.wheeldropLeft
			|| sensors.bumps_wheelDrops.wheeldropRight
			|| sensors.cliffLeft
			|| sensors.cliffFrontLeft
			|| sensors.cliffFrontRight
			|| sensors.cliffRight
	){
		// obstacle encountered
		distanceAtManeuverStart = netDistance;
		if(state != AVOID){
			// first obstacle encountered; record orientation
			angleAtManeuverStart = netAngle;
			state = AVOID;
		}

		// set avoid direction
		if(	  sensors.bumps_wheelDrops.bumpLeft
		   || sensors.bumps_wheelDrops.wheeldropLeft
		   || sensors.cliffLeft
		   || sensors.cliffFrontLeft
		){
			obstacleDirection = LEFT;
		}
		else{
			obstacleDirection = RIGHT;
		}
	}
	else if(state == AVOID && abs(netDistance - distanceAtManeuverStart) >= avoidDistance){
		// obstacle avoidance complete; reorient
		state = REORIENT;
	}
	else if(state == REORIENT && abs(netAngle - angleAtManeuverStart) <= reorientTolerance){
		// reoriented, return to drive state
		state = DRIVE;
	}
	// else, no transitions are taken

	/////////////////////////////////////////
	//             state actions           //
	/////////////////////////////////////////
	switch(state){
	case INITIAL:
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
		// in pause mode, robot should be stopped
		leftWheelSpeed = rightWheelSpeed = 0;
		break;

	case AVOID:
		// avoid an obstacle by backing up and away
		if(obstacleDirection == LEFT){
			leftWheelSpeed = -driveSpeed;
			rightWheelSpeed = -(driveSpeed >> 4);
		}
		else{
			leftWheelSpeed = -(driveSpeed >> 4);
			rightWheelSpeed = -driveSpeed;
		}
		break;

	case REORIENT:
		// set direction of rotation for shortest path
		if(angleAtManeuverStart - netAngle > 0){
			leftWheelSpeed = -reorientSpeed;
			rightWheelSpeed = reorientSpeed;
		}
		else{
			leftWheelSpeed = reorientSpeed;
			rightWheelSpeed = -reorientSpeed;
		}
		break;

	case DRIVE:
		// full speed ahead!
		leftWheelSpeed = rightWheelSpeed = driveSpeed;
		break;

	default:
		// Unknown state
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	}

	// write outputs
	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;
}
