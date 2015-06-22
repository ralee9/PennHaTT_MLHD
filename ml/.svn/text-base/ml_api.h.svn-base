/*! \file ml_api.h
  \brief This is a header file for ml_api.c, which contains the api declaration
  for both client side and server side of the Magnetic Levitation Haptic Interface API.

  \author Matthew Pucevich
  \date   Created: Jul 14, 2005  \n  Last Revision: Jul 14, 2005

  \author Joey Quansheng Liang
  \date   Updated: Jul 14, 2006

  \author Mark Dzmura
  \date   Updated: Feb 01, 2007

  \author Kei Usui
  \date   Updated: Jan 23, 2008

  <B>Supervisor:</B>                               \n 
  Dr. Ralph Hollis                                 \n\n
  <B>Location:</B>                                 \n
  Carnegie Mellon University, Robotics Institute:  \n
  Microdynamic Systems Laboratory                  \n
*/

#ifndef _ML_API_H_
#define _ML_API_H_

#include "ml_error.h" // MLHI API error codes
#include "ml_constants.h" // MLHI API custom constants
#include "ml_types.h" // MLHI API custom types
//#include "ml_api_common.h" // MLHI common non-RPC types

#ifdef __cplusplus
extern "C"{
#endif


  static const char * const CLIENT_VERSION = "1.5b";
  static const int maglev_port = 5678;

/*******************************************************/
/*******************************************************/
/***             GAIN CONTROL FUNCTIONS              ***/
/*******************************************************/
/*******************************************************/

//-----------------------------------------------//
// SIMPLE PID CONTROLLER (vector representation) //
//-----------------------------------------------//

// Resets the P, I, and D gain and FF force vectors to the system default values
int ml_ResetGainVecAxes(ml_device_handle_t dev_hdl,
			ml_gainset_type_t gainset_type );

// Sets the P, I, and D gain vectors respectively
int ml_SetGainVecAxes(ml_device_handle_t dev_hdl,
		      ml_gainset_type_t gainset_type,
		      ml_gain_vec_t gains );

// Gets the P, I, and D gain vectors respectively (into the given pointers)
int ml_GetGainVecAxes(ml_device_handle_t dev_hdl,
		      ml_gainset_type_t gainset_type,
		      ml_gain_vec_t *gains );

// Sets the P, I, and D gain values for the given (axis)
int ml_SetGainVecAxis(ml_device_handle_t dev_hdl,
		      ml_gainset_type_t gainset_type,
		      ml_axis_index_t axis,
		      ml_gain_pid_t gain );

// Gets the P, I, and D gain values for the given (axis) (into the given ptr.)
int ml_GetGainVecAxis(ml_device_handle_t dev_hdl,
		      ml_gainset_type_t gainset_type,
		      ml_axis_index_t axis,
		      ml_gain_pid_t *gain );

// Sets the P, I, and D gain values for the grasp axis
int ml_SetGainVecGrasp(ml_device_handle_t dev_hdl,
		       ml_gain_pid_t gain );

// Gets the P, I, and D gain values for the grasp axis
int ml_GetGainVecGrasp(ml_device_handle_t dev_hdl,
		       ml_gain_pid_t *gain );
		       

// Sets a single gain value for a given (axis) and (gaintype)
int ml_SetGainVecSingle(ml_device_handle_t dev_hdl,
			ml_gainset_type_t gainset_type,
			ml_axis_index_t axis,
			ml_gain_type_t gaintype,
			ml_gain_t value);

// Gets a single gain value for a given (axis) and (gaintype) (into the ptr)
int ml_GetGainVecSingle(ml_device_handle_t dev_hdl,
			ml_gainset_type_t gainset_type,
			ml_axis_index_t axis,
			ml_gain_type_t gaintype,
			ml_gain_t * value);

// Sets the limit on how fast flotor can move
int ml_SetSpeedLimits(ml_device_handle_t dev_hdl, 
			 ml_velocities_t maxspeeds);

// Gets the limit on how fast flotor can move
int ml_GetSpeedLimits(ml_device_handle_t dev_hdl,
			 ml_velocities_t* maxspeeds);

/*******************************************************/
/*******************************************************/
/***         COMMUNICATION CONTROL FUNCTIONS         ***/
/*******************************************************/
/*******************************************************/

// Initializes a communcation connection between the client and MLHI controller
int ml_Connect(ml_device_handle_t* dev_hdl,
	       const char* const addr );

// Closes a communication connection between the client and MLHI controller
int ml_Disconnect(ml_device_handle_t dev_hdl);

/*******************************************************/
/*******************************************************/
/***               CALLBACK FUNCTIONS                ***/
/*******************************************************/
/*******************************************************/

// Add Callback Step #2: Add a callback registration function declaration here.

// Specifies action to take when button state changes
int ml_RegisterCallbackButtonPressed(ml_device_handle_t dev_hdl,
				     button_state_callback_func_t buttonCallbackFunc);

// Specifies action to take when flotor violates specified edge boundary
int ml_RegisterCallbackFlotorBoundaryViolation(ml_device_handle_t dev_hdl,
					       boundary_callback_func_t boundaryCallbackFunc);

// Specifies action to take when flotor coil(s) are believed to be overheating
int ml_RegisterCallbackOvertemp(ml_device_handle_t dev_hdl,
				coil_overtemp_callback_func_t coilOvertempCallbackFunc);

// Specifies action to take if flotor signals it has shutdown
int ml_RegisterCallbackFault(ml_device_handle_t dev_hdl,
			     fault_callback_func_t faultCallbackFunc);

// Specifies action to take at the top of every servo interval
int ml_RegisterCallbackTick(ml_device_handle_t dev_hdl,
			    tick_callback_func_t tickCallbackFunc);


// Specifies action to take when button state changes
int ml_UnregisterCallbackButtonPressed(ml_device_handle_t dev_hdl);

// Specifies action to take when flotor violates specified edge boundary
int ml_UnregisterCallbackFlotorBoundaryViolation(ml_device_handle_t dev_hdl);

// Specifies action to take when flotor coil(s) are believed to be overheating
int ml_UnregisterCallbackOvertemp(ml_device_handle_t dev_hdl);

// Specifies action to take if flotor signals it has shutdown
int ml_UnregisterCallbackFault(ml_device_handle_t dev_hdl);

// Specifies action to take at the top of every servo interval
int ml_UnregisterCallbackTick(ml_device_handle_t dev_hdl);

/*******************************************************/
/*******************************************************/
/***              INTERACTION FUNCTIONS              ***/
/*******************************************************/
/*******************************************************/

// Offsets the world coordinate frame by a given transformation
int ml_SetFrame(ml_device_handle_t dev_hdl,
		ml_position_t frame);

// Gets the offset of the current world frame (into the given pointer)
int ml_GetFrame(ml_device_handle_t dev_hdl,
		ml_position_t* frame);

// Sets a new desired position for the flotor
int ml_SetDesiredPosition(ml_device_handle_t dev_hdl,
			  ml_position_t position);

// Gets the current position of the flotor
int ml_GetActualPosition(ml_device_handle_t dev_hdl,
			 ml_position_t * read_position);

// Gets the current desired position of the flotor
int ml_GetDesiredPosition(ml_device_handle_t dev_hdl,
			  ml_position_t * desired_position);

// Applies a given velocity to the flotor via interpolation from the last pos.
int ml_SetVelocity(ml_device_handle_t dev_hdl,
		   ml_velocities_t velocity);

// Gets the current flotor velocity (into the given pointer)
int ml_GetVelocity(ml_device_handle_t dev_hdl,
		   ml_velocities_t * read_velocity);

// Applies a given wrench to all 6 degrees of freedom
int ml_SetForces(ml_device_handle_t dev_hdl,
		 ml_forces_t wrench);

// Applies a given wrench to the given axis
int ml_SetForceAxis(ml_device_handle_t dev_hdl,
		    ml_axis_index_t axis,
		    ml_force_t force);

// Reads the wrench currently being applied to all axes (into the given ptr.)
int ml_GetForces(ml_device_handle_t dev_hdl,
		 ml_forces_t* wrench);

// Reads the wrench currently being applied to one of the axes (into the ptr.)
int ml_GetForceAxis(ml_device_handle_t dev_hdl,
		    ml_axis_index_t axis,
		    ml_force_t* force);

// Entirely locks motion on a given axis in its current position
int ml_LockAxis(ml_device_handle_t dev_hdl,
		ml_axis_index_t axis);

// Unlocks motion on a given axis (does nothing if already unlocked)
int ml_UnlockAxis(ml_device_handle_t dev_hdl,
		  ml_axis_index_t axis);

// Restricts motion on a specified axis to within specified bounds
int ml_ConstrainAxis(ml_device_handle_t dev_hdl,
		     ml_axis_index_t axis,
		     ml_coord_t minpos,
		     ml_coord_t maxpos);

// Returns axis mode for each axis.
int ml_GetAxisModes(ml_device_handle_t dev_hdl,
		    ml_axis_mode_vec_t * axis_mode_vec);

// Finds the orientation of the gravity vector in the current coordinate frame.
int ml_FindGravity(ml_device_handle_t dev_hdl,
		   ml_forces_t *gravity);

// Inputs a new vector to use as the gravity vector
int ml_SetGravity(ml_device_handle_t dev_hdl,
		  ml_forces_t garvity);

// Reads the gravity vector currently being used by the MLHI controller
int ml_GetGravity(ml_device_handle_t dev_hdl,
		  ml_forces_t* read_gravity);

// Causes the MLHI to resist gravity along the currently set gravity vector
int ml_DefyGravity(ml_device_handle_t dev_hdl);

// Transitions the flotor from an idle state to an initial, floating state.
int ml_Takeoff(ml_device_handle_t dev_hdl);

// Transitions the flotor from a floating state to an idle state.
int ml_Land(ml_device_handle_t dev_hdl);

// Sends a shutdown command to the MLHI controller
int ml_Shutdown(ml_device_handle_t dev_hdl);

// Get buttons status on the handle
int ml_GetButtonStatus(ml_device_handle_t dev_hdl,
		       ml_button_t *buton);


/*******************************************************/
/*******************************************************/
/***                UTILITY FUNCTIONS                ***/
/*******************************************************/
/*******************************************************/

// Reads the estimated temperatures of the six coils
int ml_GetTemp(ml_device_handle_t dev_hdl,
	       ml_temps_t * read_temp);

// Returns the prevailing servo frequency.
int ml_GetServoFrequency(ml_device_handle_t dev_hdl,
			float * servo_frequency);

// Sets the prevailing servo frequency.
int ml_SetServoFrequency(ml_device_handle_t dev_hdl,
			float servo_frequency);

// Outputs specified voltage to Aux Diagnostic Output
int ml_SetAuxDac(ml_device_handle_t dev_hdl,
		 float voltage);

// Gets server and client api version
int ml_GetVersion(ml_device_handle_t dev_hdl,
		  char * client_version,
		  char * server_version);

/*******************************************************/
/*******************************************************/
/***            DIRECT HARDWARE FUNCTIONS            ***/
/*******************************************************/
/*******************************************************/

// Set the currents to output to each of the six coils
int ml_SetCurrents(ml_device_handle_t dev_hdl,
		   ml_currents_t client_currents);

// Reads the currents currently being applied to the coils (into the ptr.)
int ml_GetCurrents(ml_device_handle_t dev_hdl,
		   ml_currents_t* read_currents);

// Sets the limit on how much current can be applied to each coil
int ml_SetCurrentLimits(ml_device_handle_t dev_hdl,
			ml_currents_t maxcurrents);

// Gets the maximum current that can be applied to each coil (into the ptr.)
int ml_GetCurrentLimits(ml_device_handle_t dev_hdl,
			ml_currents_t* maxcurrents);

// Sets the current to output to the grasp axis
int ml_SetGraspCurrent(ml_device_handle_t dev_hdl,
		       ml_current_t current);

// Gets the current being applied to the grasp axis
int ml_GetGraspCurrent(ml_device_handle_t dev_hdl,
		       ml_current_t* current);

// Sets the maximum current that can be applied to the grasp axis
int ml_SetGraspCurrentLimit(ml_device_handle_t dev_hdl,
			    ml_current_t maxcurrent);

// Gets the maximum current that can be applied to the grasp axis
int ml_GetGraspCurrentLimit(ml_device_handle_t dev_hdl,
			    ml_current_t* maxcurrent);

// Gets the maximum position of the LED position sensors
int ml_GetLatCellLimit(ml_device_handle_t dev_hdl,
		       float *limit);

// Sets a calibration offset for the LED position sensors
int ml_SetLatCellOffset(ml_device_handle_t dev_hdl, 
			ml_sensor_offset_t cella,
			ml_sensor_offset_t cellb,
			ml_sensor_offset_t cellc);

// Gets the calibration offset for the LED position sensors
int ml_GetLatCellOffset(ml_device_handle_t dev_hdl, 
			ml_sensor_offset_t* cella,
			ml_sensor_offset_t* cellb,
			ml_sensor_offset_t* cellc);

// Gets the raw cell position values and places them into the given addresses
int ml_GetCellPosition(ml_device_handle_t dev_hdl, 
		       ml_sensor_t* cella,
		       ml_sensor_t* cellb,
		       ml_sensor_t* cellc);

// Gets the raw voltages coming off of the three lat cells
int ml_GetRawCellData(ml_device_handle_t dev_hdl,
		      ml_sensor_raw_data_t *sensorRaw);

// Read ADC values and store them into the buffer
int ml_RecordVoltages(ml_device_handle_t dev_hdl);

// Update the flotor position information
int ml_UpdatePosition(ml_device_handle_t dev_hdl);

// get radius of bounding sphere
int ml_GetBoundaryRadius(ml_device_handle_t dev_hdl,
			 float * radius);

// set radius of bounding sphere
int ml_SetBoundaryRadius(ml_device_handle_t dev_hdl,
			 float radius);

int ml_GetFault(ml_device_handle_t dev_hdl,
		ml_fault_t * fault);
//bool * fault);

int ml_ResetFault(ml_device_handle_t dev_hdl);

#ifdef __cplusplus
}
#endif

#endif	// __ML_API_H__
