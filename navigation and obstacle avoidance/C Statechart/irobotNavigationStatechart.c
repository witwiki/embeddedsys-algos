/*
@witwiki #29052014
Finite State Machine for the iRobot Create.
As part of the CyberPhysical Systems program GoBearsX
*/

#include "irobotNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>

/// Program States
typedef enum{
	INITIAL = 0,						///< Initial state
	PAUSE_WAIT_BUTTON_RELEASE,			///< Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,			///< Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE,		///< Paused; pause button pressed down, wait until released before returning to previous state
	DRIVE,								///< Drive straight
	DRIVE2,								///< Second Drive State
	DRIVE3,								///< Third Drive State
	DRIVE4,								///< Fourth Drive State
	TURN,								///< Turn right
	TURN2,								///< Second Turn right State
	TURN_LEFT,							///< Turn Left State
	TURN_LEFT2,							///< Second Turn Left State
} robotState_t;

void irobotNavigationStatechart(
	const int32_t 				netDistance,
	const int32_t 				netAngle,
	const irobotSensorGroup6_t 	sensors,
	const accelerometer_t 		accel,
	const bool					isSimulator,
	int16_t * const 			pRightWheelSpeed,
	int16_t * const 			pLeftWheelSpeed
){
	// local state
	static robotState_t 		state = INITIAL;				// current program state
	static robotState_t			unpausedState = DRIVE;			// state history for pause region
	static int32_t				distanceAtManeuverStart = 0;	// distance robot had travelled when a maneuver begins, in mm
	static int32_t				angleAtManeuverStart = 0;		// angle through which the robot had turned when a maneuver begins, in deg

	// outputs
	int16_t						leftWheelSpeed = 0;				// speed of the left wheel, in mm/s
	int16_t						rightWheelSpeed = 0;			// speed of the right wheel, in mm/s

	//*****************************************************
	// state data - process inputs                        *
	//*****************************************************
	
		
	// obstacle avoidance algorithm psuedocode
	/*
	when driving straight,
	if a sensor flares up/glows
		then change state to turn for a say 90 degrees
		then change state to drive for say 50 mm
		then change state to turn for -90 degrees
		then change state to drive for the rest
	repeat these steps if sensors flares up/glows till you reach the end state

	state = TURN;
	netAngle - angleAtManeuverStart = 90;

	*/

	
	//*****************************************************
	// state transition - pause region (highest priority) *
	//*****************************************************
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
	//*************************************
	// state transition - run region      *
	//*************************************
	/*else if (sensors.bumps_wheelDrops.bumpLeft != 0){
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = TURN;
	}*/
	else if (sensors.wallSignal >= 0 && sensors.wall == 1 || sensors.bumps_wheelDrops.bumpLeft != 0){
		angleAtManeuverStart = netAngle;					//	Changes the current state (aka netAngle) to the initial (aka angleAtManeuverStart)
		distanceAtManeuverStart = netDistance;				//	Changes the current state (aka netDistance) to the initial (aka distanceAtManeuverStart)
		state = TURN;
	}
	else if (state == TURN && abs(netAngle - angleAtManeuverStart) >= 79){
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = DRIVE;
	}
	else if (state == DRIVE && abs(netDistance - distanceAtManeuverStart) >= 800){
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = TURN_LEFT;
	}
	else if (state == TURN_LEFT && abs(netAngle - angleAtManeuverStart) >= 88){
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = DRIVE2;
	}
	else if (state == DRIVE2 && abs(netDistance - distanceAtManeuverStart) >= 1500){
		angleAtManeuverStart = netAngle;					
		distanceAtManeuverStart = netDistance;				
		state = TURN_LEFT2;
	}
	else if (state == TURN_LEFT2 && abs(netAngle - angleAtManeuverStart) >= 89){
		angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = DRIVE3;
	}
	else if (state == DRIVE3 && abs(netDistance - distanceAtManeuverStart) >= 1000){
		//angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = TURN2;
	}
	else if (state == TURN2 && abs(netAngle - angleAtManeuverStart) >= 50){
		//angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		state = DRIVE4;
	}
	else if (state == DRIVE4 && abs(netDistance - distanceAtManeuverStart) >= 9000){
		//angleAtManeuverStart = netAngle;
		distanceAtManeuverStart = netDistance;
		//state = TURN3;
	}

	// else, no transitions are taken

	//*****************
	//* state actions *
	//*****************
	switch(state){
	case INITIAL:
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
		// in pause mode, robot should be stopped
		leftWheelSpeed = rightWheelSpeed = 0;
		break;

	case DRIVE:
		// full speed ahead!
		leftWheelSpeed = rightWheelSpeed = 200;
		break;

	case DRIVE2:
		// full speed ahead!
		leftWheelSpeed = rightWheelSpeed = 200;
		break;

	case DRIVE3:
		// full speed ahead!
		leftWheelSpeed = rightWheelSpeed = 200;
		break;

	case DRIVE4:
		// full speed ahead!
		leftWheelSpeed = rightWheelSpeed = 200;
		break;

	case TURN:
		leftWheelSpeed = 100;
		rightWheelSpeed = -leftWheelSpeed;
		break;

	case TURN2:
		leftWheelSpeed = 100;
		rightWheelSpeed = -leftWheelSpeed;
		break;

	case TURN_LEFT:
		leftWheelSpeed = -100;
		rightWheelSpeed = -leftWheelSpeed;
		break;

	case TURN_LEFT2:
		leftWheelSpeed = -100;
		rightWheelSpeed = -leftWheelSpeed;
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
