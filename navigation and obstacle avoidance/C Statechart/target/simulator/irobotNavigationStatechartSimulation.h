/** \file irobotNavigationStatechartSimulation.h
 *
 * Inteface between the LabVIEW Robotics Environment Simulator
 * (either LabVIEW or libsimulator.dll) and the controller.
 *
 * \author Jeff C. Jensen
 * \date 2014-01-09
 * \copyright Copyright (C) 2013, Jeff C. Jensen, Edward A. Lee, and Sanjit A. Seshia.
 * 			  This software accompanies An Introductory Lab in Embedded and Cyber-Physical Systems
 * 			  and is licensed under a Creative Commons Attribution-NonCommercial-NoDerivs 3.0
 * 			  Unported License. See http://leeseshia.org/lab.
 */

#ifndef IROBOTNAVIGATIONSTATECHARTSIMULATION_H_
#define IROBOTNAVIGATIONSTATECHARTSIMULATION_H_

#ifdef LIBSTATECHARTEXAMPLE_EXPORTS
	#define LIBSTATECHARTEXAMPLE_EXP	__declspec(dllexport) __cdecl
#else
	#define LIBSTATECHARTEXAMPLE_EXP	__declspec(dllimport) __cdecl
#endif

#include <stdint.h>

/// This function is part of the hardware abstraction layer, and serves as the
/// interface between LabVIEW Robotics Environment Simulator and the architecture-
/// independent statechart. By design, this is the signature that any
/// build tool (Visual Studio, LabVIEW, MATLAB, Ptolemy, etc.) must match to be called
/// by the simulator.
///
/// \warning Any changes made to this function prototype must be reflected in the LabVIEW
/// Call Library Function Node configuration in the simulator, as well as any tool that
/// produces a Statechart controller to be used by the simulator. Changes here must be
/// refactored into all tools (simulator and statechart builders) and should be avoided.
///
/// \return LabVIEW error code
int32_t LIBSTATECHARTEXAMPLE_EXP irobotNavigationStatechartSimulation(
	const int32_t 			netDistance,		///< [in] net distance, in mm
	const int32_t 			netAngle,			///< [in] net angle, in deg
	const uint8_t * const 	sensorStream,		///< [in] simulated sensor stream (1 packet)
	const int32_t			sensorStreamSize,	///< [in] sensor stream size, in bytes
	const double * const	accelAxes,			///< [in] accelerometer, in g
	const int32_t			accelAxesSize,		///< [in] accelAxes size; should always be 3
	int16_t * const 		pRightWheelSpeed,	///< [out] right wheel speed, in mm/s
	int16_t * const 		pLeftWheelSpeed		///< [out] left wheel speed, in mm/s
);


#endif // IROBOTNAVIGATIONSTATECHARTSIMULATION_H_
