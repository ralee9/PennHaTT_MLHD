/*! \file ml_api.c
  \brief This source file contains the "client side" of the Magnetic Levitation
  Haptic Interface API (that which runs on the client computer system).

  \author Matthew Pucevich
  \date   Created: June 01, 2005  \n  Last Revision: July 6, 2005

  \author Joey Quansheng Liang
  \date   Updated: June 01, 2006

  \author Mark Dzmura
  \date   Updated: Feb 01, 2007

  \author Kei Usui
  \date   Updated: Feb 11, 2008

  <B>Supervisor:</B>                               \n
  Dr. Ralph Hollis                                 \n\n
  <B>Location:</B>                                 \n
  Carnegie Mellon University, Robotics Institute:  \n
  Microdynamic Systems Laboratory                 \n
*/

/*! \mainpage Magnetic Levitation Haptic Interface Application Program Interface Reference Manual
  \section intro_sec Introduction

  This document provides detailed information on an Application
  Program Interface (API) for the Magnetic Levitation Haptic Interface
  (MLHI) - the MLHI API. The purpose of this API is to provide a
  comprehensive set of access functions and foundational tools to
  allow client program to interact with the MLHI device, and to allow
  MLHI users to construct the haptic component of their virtual environment.

  \subsection Scope MLHI API Scope

  The relationship of the MLHI API to other components of a generic visio-haptic system is
  illustrated in Fig. 1. As can be seen from the figure, the MLHI API deals only with the
  haptic device hardware. Other components, shown by dashed boxes, must be supplied by the
  user. Here, the dynamic virtual environment is a physically-based model and simulation for
  a specific visio-haptic application. Visual aspects of this virtual environment must be
  handled by user-supplied rendering techniques, while haptic aspects must be handled by
  user-supplied haptic rendering techniques.

  <!--   \image html hapticdiagram.jpg -->
  \image latex hapticDiagram.eps "Relationship of MLHI API to other components of a visio-haptic system: software within the dashed boxes must be supplied by the user." width=6in

  \section coderesidence_sec System Layout

  The typical hardware setup for the MLHI system has three parts, the MLHI device head,
  which is connected to the MLHI controller, which is itself connected to the user's
  client computer system (running Windows or some Linux derivative). See diagram below:

  <!--   \image html HapBoxSetup.jpg -->
  \image latex maglevSystem.eps "Typical MLHI system setup" width=5in

  \subsection HapticDevice The MLHI Device Head

  The MLHI device head itself is connected via sensor and actuator cables to the MLHI
  controller. Over this link, the device reports state and receives commands directly from
  the MLHI controller.

  \subsection ControlBox The MLHI Controller:

  The MLHI controller handles workhorse routines that keep the flotor floating,
  performs calculations that convert sensor data into useful kinematics information,
  runs a controller (PID by default), and accepts commands from the client
  machine conforming to this API.

  \subsection Usermachine The Client Computer System:

  The client machine is a PC running either the Windows or Linux operating
  system.  In normal circumstances, haptic applications will be
  running on this client machine.  The machine is connected to the
  MLHI controller via either an ethernet interface, or via USB 2.0. The
  client machine interacts with the haptic device through the MLHI
  controller, sending API commands to it, and receiving kinematics and
  state data back from it. Control of the communication link can be
  found in the "Communication Control Functions" section of the API.

  For an Ethernet connection between the client system and the MLHI
  controller, a dedicated Ethernet card must be installed in the user's
  system, and a special Ethernet crossover cable must be used.

  For a USB 2.0 connection between the user's system and the MLHI controller,
  a special USB 2.0 host-to-host adapter cable must be used.

  \subsection ControlApplications Running applications from the Haptic Control Box:

  It is possible to run applications on the MLHI controller itself, in which case
  the client machine is technically unnecessary. However, given that the MLHI controller is
  already busy with the variety of other tasks needed to run the haptic hardware, it may
  not have the computing resources available to run a complicated application. Note that
  the processes responsible for keeping the haptic device stable run on the MLHI controller
  at maximum priority. Therefore if the computing resources of the MLHI controller are
  overwhelmed by an application, the device ought to remain stable, though the
  application will perform poorly.

  To run applications from the MLHI controller:
  First copy the files and/or executables of the application onto the MLHI controller,
  Second, when making the function call to connect to the MLHI controller (the
  \c ml_Connect() API function), set the protocol argument to \c PROTOCOL_UDP,
  and the address argument to ADDRESS_SELF.

  \section Coordinates MLHI User Coordinate System

  \htmlonly  The MLHI uses a right-handed  <i>x, y, z, roll, pitch, yaw</i>
  coordinate system. In the device's standard coordinate frame, the notch in the stator
  shell indicates the direction of the positive <i>y</i> axis as shown in the figure
  below. From this, the position of all other axes can be derived (roll is rotation about
  the <i>y</i> axis, pitch about the <i>x</i>, and yaw about the <i>z</i>). The origin is
  located at the center of the flotor's range of motion. \endhtmlonly

  \latexonly  All coordinate frames are right handed. The flotor frame $\cal F$ is attached
  to the handle and has its origin at the center of the hemispherical flotor. the stator
  frame $\cal S$ is attached to the stator and has its origin at the center of the
  hemispherical stators. The $y$ axis of $\cal S$ points toward the fiducial mark feature
  cut into the stator housing. At position {0,0,0,0,0,0} $\cal F$ and $\cal S$ are
  coincident. Roll, pitch, and yaw of the flotor frame are defined in a right hand sense.
  Positive roll $\phi$ is about $y$, positive pitch $\theta$ is about $x$, and positive
  yaw $\psi$ is about $z$. The client may define a frame $\cal U$ that is translated and
  rotated with respect to the stator frame $\cal S$ as shown. This is done by
  executing {\bf ml\_SetFrame } with appropriate arguments. After this is done, all
  operation of the device will be with respect to the new frame $\cal U$. \endlatexonly

  \image latex hapticFrames.eps "MLHI coordinate frame orientation" width=10cm

  \subsection CoordframeChanging Changing the Coordinate Frame:

  Through the use of the \c ml_SetFrame function, it is possible to change the position
  of the user coordinate frame. The \c ml_SetFrame function takes an <i>x, y,</i> and
  <i>z</i> translation, as well as a <i>roll, pitch,</i> and <i>yaw</i> rotation. These
  describe the offset of the desired new coordinate frame from the <b>default</b> (NOT
  the current) coordinate frame.

  \section graspaxis The Grasp Axis:

  Implemented in this API is a seventh degree of freedom for the haptic device,
  referred to as the <i>grasp</i> axis. The grasp axis is an optional addition to the
  device that allows for the use of a gripping mechanism, in addition to the other
  six positional and rotational axes.

  \section Organization API Organization
  The functions in this API are broken down into five main categories: Setup functions,
  Communication control functions, Callback functions, Interaction functions,
  Utility functions, and Direct Hardware functions.

  \subsection IntroCommfs Communication Control Functions

  Functions in this category deal with initializing and configuring the communication
  connection between the application and one or several MLHI controllerss.

  \subsection IntroGainfs Parameter Manipulation Functions:

  Functions in this category deal with manipulation of PID controller parameters, such as gain values, by means of <i>set</i> and <i>get</i> operations.

  \subsection IntroInteractionfs Haptic Interaction Functions:

  These are the routines which deal with running a haptic simulation;
  such as setting/reading the flotor position, setting/reading the
  wrench (forces) being applied to it, altering the user coordinate
  frame, and more.

  \subsection IntroCallbackfs Callback Manipulation Functions:

  These routines allow client programs to register and unregister functions to be called
  when certain events occur, such as when a button on the haptic device is pressed, or
  when the flotor is overheating.

  \subsection IntroUtilityfs Utility Functions:

  The utility functions are those which deal with calibrating and monitoring the
  status of the haptic device. Functions in this section perform tasks ilke applying
  a calibration offset to the lat cell position sensors, or checking the estimated
  coil temperatures.

  \subsection IntroDirectfs Direct Hardware Functions:

  Provided for completeness' sake, these functions allow very low level access to
  the hardware of the MLHI device; which can be useful for hardware
  debugging, or possibly some interesting simulations. Note that these functions
  have the capacity to give commands and/or define limits that may be harmful to
  the haptic device hardware, as such these functions are ``use at your own risk.''

  \section Units Units

  Unless otherwise stated explicitly in function documentation, the MLHI device
  uses the following SI units for its variables:

  \li Force: newtons (N)
  \li Torque: newton-meters (N-m)
  \li Length: meters (m)
  \li Angle: radians (rad)
  \li Time: seconds (s)
  \li Temperature: degrees Celsius (C)
  \li Potential: volts (V)
  \li Current: amperes (A)
*/

/* Insert into .h file, and other "to be moved" code */

/*
 */


#ifdef WIN32
#include <winsock.h>
#else
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ml_api.h"
#include "ml_api_1.h"
#include "ml_api_callback.h"
#include "ml_serialization.h"
#include "ml_communication.h"


/** \defgroup CommFunctions Haptic Interaction Functions

    \brief These functions support the establishment, maintenance, and
    termination of communication between the client system and the
    MLHI controller.

    As mentioned in the introduction, the MLHI system
    typically consists of three parts, the MLHI head (device), the
    MLHI controller, and the client system (See fig 1.1 of the
    introduction for an illustration). In addition to relaying rapid,
    low level instructions to the hardware of the MLHI head and
    running the PID controller; the MLHI controller functions as a
    communication bridge between the client system and the MLHI head.

    Naturally, this requires transmitting flotor state and other
    values between the client system and the MLHI controller. The
    functions in this section pertain to the setup and control of that
    data flow between the client system and the MLHI controller. To
    create a connection between a client system and a MLHI controller
    across which data can flow, the client must first call \c
    ml_Connect().

    <B>Connection Modes</B>
    There are two different types of connection between a client system and a MLHI controller:
    Command mode and Receive Only mode (The desired connection mode is given as the second
    argument of \c ml_Connect()).

    <i>Command Mode:</i>
    Only one client system may be connected to the MLHI controller in command mode at a time.
    Command mode allows the client system to issue flotor commands to the MLHI controller and to
    receive state information from the MLHI controller (Basically allows the client to call
    both set and get types of routines).

    <i>Receive Only Mode:</i>
    There is technically no limit to the number of client systems which may connect to the MLHI
    controller in Receive Only mode, however at some point processing resources or network
    bandwidth limits will be encountered. Receive Only mode only allows the client system to
    request state information from the MLHI controller, and forbids the client system from
    issuing commands which will alter the state of the device (basically allows "get"
    functions, but forbids "set" functions). Receive Only mode allows clients to run diagnostic
    or monitoring functions alongside their simulation functions.

    A client can attempt to change the connection mode of an existing connection using the
    \c ml_ChangeCommMode() API function. Note that a client can not switch a connection to
    a given MLHI controller to command mode unless that MLHI controller currently has no
    other clients connected in command mode. (The MLHI controller grants command mode
    on a first come first serve basis).

    <B>Device Handles</B>
    The \c ml_Connect() function also returns a device handle number through its fourth argument.
    These device_handle_t's are used in most API function calls to specify to which MLHI device a
    given API function call applies. API calls wich require a device handle will take it as
    their first argument.

/** @{ */


/** \defgroup GainFunctions High Level Functions
    \brief These functions are used to send high level commands to the MLHI controller.
    
    - \c ml_SetGain*, \c ml_GetGain* and \c ml_ResetGainVecAxes() functions are used to
    manipulate the gain settings of the haptic device. The MLHI device
    provides built-in PIDF (proportional, integral, derivative, feedforward) controllers.
    - \c ml_*Gravity functions are used to manipulate anti-gravity feedforward forces. 
    - \c ml_Takeoff() is used to smoothly take the flotor off to the origin and 
    \c ml_Land() is used to smoothly land the flotor to the bottom.
    - \c ml_SetBoundaryRadius() and \c ml_GetBoundaryRadius() functions are used to manipulate the boundary sphere
    which prevents the flotor to go out from the sensor range or collide with the mechanical boundaries of the device.
    - \c ml_SetFrame() and \c ml_GetFrame() functions are used to manipulate the user-defined coordinate frame 
    when sensing and actuating the flotor.
    - \c ml_LockAxis(), \c ml_UnlockAxis() and \c ml_ConstrainAxis() are used to manipulate the control mode for each of the six axes.
  
  */

/** @{ */

/*! \fn int ml_Takeoff(ml_device_handle_t dev_hdl)
  \brief Transitions the flotor from its current state to the default position and orientation 
  with the default set of gains.

  ml_Takeoff can be used to carry out the process of initially levitating the flotor from a
  powered-down idle state. ml_Takeoff smoothly changes the desired position and the gains while blocking, 
  not returning until completed. 
  After taking off, the flotor will be servoed to position/orientation of [0,0,0,0,0,0].
  This function takes one argument \a dev_hdl. This
  argument is the device handle of the MLHI controller at which this command is directed.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_FLOTOR_TAKEN_OFF if the flotor has already taken off, \n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_Land()
*/


int ml_Takeoff(ml_device_handle_t dev_hdl)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_TAKEOFF,
			      NULL, NULL);
}

/*! \fn int ml_Land(ml_device_handle_t dev_hdl)
  \brief Transitions the flotor from a floating ready state to a powered-down
  idle state.

  ml_Land is used to carry out the process of landing the flotor by gradually zeroing 
  all 4 gains along along all 6 axes. ml_Land blocks from returning until completed. 
  This function takes one argument \a dev_hdl, 
  which is the device handle of the MLHI controller at which this command is directed.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_Takeoff()
*/

int ml_Land(ml_device_handle_t dev_hdl)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_LAND,
			      NULL, NULL);
}

/*! \fn int ml_ResetGainVecAxes(ml_device_handle_t dev_hdl,
  ml_gainset_type_t gainset_type)

  \brief Resets all axis gain values (<i>x, y, z, roll, pitch, yaw</i>) to the system
  default values smoothly over a few seconds for the PIDF controller.

  ml_ResetGainVecAxes is used to reset the PIDF gain values to the system default values.
  This function takes 2 arguments. The argument \a dev_hdl
  is the device handle of the MLHI controller to which this command is directed.  The argument
  \a gainset_type is the gainset type to be reset.

  If the given \a dev_hdl is invalid, then the function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If any error occurs (the function returns something besides \c ML_STATUS_OK), then
  no gain values will be changed.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param gainset_type The gainset to be reset.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection, or \n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_SetGainVecAxes(), \c ml_GetGainVecAxes()
*/

int ml_ResetGainVecAxes(ml_device_handle_t dev_hdl,
			ml_gainset_type_t gainset_type)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_RESETGAINVECAXES,
			      &gainset_type, NULL);
}


/*! \fn int ml_SetGainVecAxes(ml_device_handle_t dev_hdl,
  ml_gainset_type_t gainset_type,
  ml_gain_vec_t gains)

  \brief Sets all axis gain values (<i>x, y, z, roll, pitch, yaw</i>) immediately for
  the PIDF controller.

  ml_SetGainVecAxes is used to set all 4 gains for all 6 axes for the  PIDF controller.
  This function takes 3 arguments. The first argument \a dev_hdl
  is the device handle of the MLHI controller to which this command is directed.
  The second argument \a gainset_type indicates which of the four gain sets will be modified.
  The third argument \a gains contains a vector of 6 elements, each containing four values: 
  P, I, D, and feedforward, representing each of the 24 values in a gainset.

  For translational axes, units are 
  <i> P(N/m), I(N/ms), D(Ns/m), F(N) </i>. 
  For rotational axes, units are 
  <i> P(Nm/rad), I(Nm/rads), D(Nms/rad), F(Nm). </i>

  If the given \a dev_hdl is invalid, then the function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If any error occurs (the function returns something besides \c ML_STATUS_OK), then
  no gain values will be changed.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param gainset_type The gainset to be used.
  \param gains The PIDF gains on six axes. 
  
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection, or \n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetGainVecAxes(), \c ml_ResetGainVecAxes(), \c ml_SetGainVecAxis(), \c ml_GetGainVecAxis(), 
  \c ml_SetGainVecSingle(), \c ml_GetGainVecSingle()
*/

int ml_SetGainVecAxes(ml_device_handle_t dev_hdl,
		      ml_gainset_type_t gainset_type,
		      ml_gain_vec_t gains)
{
  ml_set_gain_vec_axes_t args_in;
  args_in.gainset_type = gainset_type;
  memcpy(&(args_in.gains), &gains, sizeof(ml_gain_vec_t));
  
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_SETGAINVECAXES,
			      &args_in, NULL);
}


/*! \fn int ml_GetGainVecAxes( ml_device_handle_t dev_hdl,
		       ml_gainset_type_t gainset_type,
		       ml_gain_vec_t *gains)
  \brief Gets all axis gain values (<i>x, y, z, roll, pitch, yaw</i>) at once for
  the PIDF controller.

  ml_GetGainVecAxes is used to read back all four gain
  vectors (P, I, D, and F) used by the PIDF controller (24 values). This function takes three
  arguments. The first argument \a dev_hdl is the device handle of the MLHI controller to which
  this command is directed.
  The second argument, \a gainset_type, indicates which of the four gain sets will be read.
  The third argument, \a gains, points to a result vector of 6 elements, each containing four values: 
  P, I, D, and feedforward, representing each of the 24 values in a gainset.

  For translational axes, units are 
  <i> P(N/m), I(N/ms), D(Ns/m), F(N) </i>. 
  For rotational axes, units are 
  <i> P(Nm/rad), I(Nm/rads), D(Nms/rad), F(Nm). </i>

  If the given \a dev_hdl is invalid, then this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE and no values will be read.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param gainset_type The gainset whos gains are read.
  \param gains The PIDF gains on six axes.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetGainVecAxes(), \c ml_ResetGainVecAxes(), \c ml_SetGainVecAxis(), \c ml_GetGainVecAxis(), 
  \c ml_SetGainVecSingle(), \c ml_GetGainVecSingle()

*/

int ml_GetGainVecAxes(ml_device_handle_t dev_hdl,
		      ml_gainset_type_t gainset_type,
		      ml_gain_vec_t * gains)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETGAINVECAXES,
			      &gainset_type, gains);
}


/*! \fn int ml_SetGainVecAxis(ml_device_handle_t dev_hdl,
  ml_gainset_type_t gainset_type,
  ml_axis_index_t axis,
  ml_gain_pid_t pidgains)
  \brief Sets the proportional, integral, and derivative gains and feedforward force for one axis.

  ml_SetGainVecAxis is used to set the proportional, integral, and derivative gain
  values and feedforward force for one given axis. This function takes four arguments. The first argument
  \a dev_hdl is the device handle of the MLHI controller to which this command is directed.
  The second argument, \a gainset_type, indicates which of the four gain sets will be modified.
  The third argument \a axis designates which axis's gain values are to be set. The
  fourth argument \a pidgains is a pointer to a 4-element array containing the new
  proportional, integral, and derivative gain values, 
  and the feedforward force, respectively, for the given \a axis.

  For translational axes, units are 
  <i> P(N/m), I(N/ms), D(Ns/m), F(N) </i>. 
  For rotational axes, units are 
  <i> P(Nm/rad), I(Nm/rads), D(Nms/rad), F(Nm). </i>

  If the given \a dev_hdl is invalid, then this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If any error occurs (this function returns something besides \c ML_STATUS_OK, then
  no gain values will be changed.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param gainset_type The gainset to be used.
  \param axis The axis whose gains will be set
  \param pidgains The new gain values for the given axis as a 4-element
  array (proportional, integral, derivative, feedforward)
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetGainVecAxis(), \c ml_SetGainVecAxes(), \c ml_GetGainVecAxes(), \c ml_ResetGainVecAxes(),
  \c ml_SetGainVecSingle(), \c ml_GetGainVecSingle()
*/

int ml_SetGainVecAxis(ml_device_handle_t dev_hdl,
		      ml_gainset_type_t gainset_type,
		      ml_axis_index_t axis,
		      ml_gain_pid_t pidgains)
{
  ml_set_gain_vec_axis_t args_in;

  if (axis < ML_AXIS_X || axis > ML_AXIS_Z_ROT)
    return ML_STATUS_DATA_AXIS_INVALID;
  
  args_in.axis  = axis;
  args_in.gainset_type = gainset_type;
  memcpy(&args_in.gain, &pidgains, sizeof(ml_gain_pid_t));
  
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_SETGAINVECAXIS,
			      &args_in, NULL);
}

/*! \fn int ml_GetGainVecAxis(ml_device_handle_t dev_hdl,
  ml_gainset_type_t gainset_type,
  ml_axis_index_t axis,
  ml_gain_pid_t * pidgains)

  \brief Gets the proportional, integral, and derivative gains and the feedforward force for one axis.

  ml_GetGainVecAxis is used to read the proportional, integral, and derivative gain
  values and the feedforward force for one given axis. This function takes four arguments. The first argument
  \a dev_hdl is the device handle of the MLHI controller to which this command is directed.
  The second argument, \a gainset_type, indicates which of the four gain sets will be read.
  The third argument \a axis designates which axis's gain values are to be read.
  The fourth argument \a pidgains, is a pointer to a 4-element array into which the
  read proportional, integral, and derivative gain values, and the feedforward force, respectively, will be written.

  For translational axes, units are 
  <i> P(N/m), I(N/ms), D(Ns/m), F(N) </i>. 
  For rotational axes, units are 
  <i> P(Nm/rad), I(Nm/rads), D(Nms/rad), F(Nm). </i>

  If the given \a dev_hdl is invalid, then this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE and no values will be read.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param gainset_type The gainset to be used.
  \param axis The axis whose gains will be gotten.
  \param pidgains Array into which the current proportional, integral, and
  derivative gain values and feedforward force value for one of the six axes (<i>x, y, z, roll, pitch, yaw</i>)
  will be written.
  \return \c ML_STATUS_OK ( 0 ) upon success, or \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetGainVecAxis(), \c ml_SetGainVecAxes(), \c ml_GetGainVecAxes(), \c ml_ResetGainVecAxes(),
  \c ml_SetGainVecSingle(), \c ml_GetGainVecSingle()
*/

int ml_GetGainVecAxis(ml_device_handle_t dev_hdl,
		      ml_gainset_type_t gainset_type,
		      ml_axis_index_t axis,
		      ml_gain_pid_t* pidgains)
{
  ml_get_gain_vec_axis_t args_in;
  if (axis < ML_AXIS_X || axis > ML_AXIS_Z_ROT)
    return ML_STATUS_DATA_AXIS_INVALID;
  
  args_in.axis = axis;
  args_in.gainset_type = gainset_type;
  
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETGAINVECAXIS,
			      &args_in, pidgains);
}

/*! \fn int ml_SetGainVecGrasp(ml_device_handle_t dev_hdl, ml_gain_pid_t pidgains)
  \brief Sets the proportional, integral, and derivative gains for the grasp axis.
  \todo Not Yet Implemented.

  ml_SetGainVecGrasp is used to set the proportional, integral, and derivative gain
  values and feedforward force for the grasp axis. This function takes two arguments. The first argument
  \a dev_hdl is the device handle of the MLHI controller to which this command is directed.
  The second argument \a pidgains is a pointer to a 4-element array which contains
  the new proportional, integral, and derivative gain values, respectively.
  (The feedforward force element might not be used for the grasp axis.)

  Units are 
  <i> P(N/m), I(N/ms), D(Ns/m), F(N) </i>. 

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If any error occurs (this function returns something besides \c ML_STATUS_OK, then
  no gain values will be changed.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param pidgains The new gain values for the given axis as a 4-element array (position,
  integral, derivative)
  \return ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetGainVecGrasp()
*/

int ml_SetGainVecGrasp(ml_device_handle_t dev_hdl,
		       ml_gain_pid_t pidgains)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_SETGAINVECGRASP,
			      &pidgains, NULL);
}

/*! \fn int ml_GetGainVecGrasp(ml_device_handle_t dev_hdl, ml_gain_pid_t* pidgains)
  \brief Gets the proportional, integral, and derivative gains for the grasp axis.
  \todo Not Yet Implemented.

  ml_GetGainVecGrasp is used to get the current proportional, integral, and derivative
  gain values and feedforward force for the grasp axis. This function takes two agruments. The first argument
  \a dev_hdl is the device handle of the MLHI controller to which this command is directed. The
  second argument \a pidgains is a pointer to a 4-element array into which the current
  proportional, integral, and derivative gain values, respectively, will be written.
  (The feedforward force element might not be used for the grasp axis.)

  Units are
  <i> P(N/m), I(N/ms), D(Ns/m), F(N) </i>. 

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param pidgains Array into which the current proportional, integral, and derivative gain
  values for the grasp axis will be written
  \return ML_STATUS_OK ( 0 ) upon success, or \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetGainVecGrasp()
*/

int ml_GetGainVecGrasp(ml_device_handle_t dev_hdl,
		       ml_gain_pid_t* pidgains)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETGAINVECGRASP,
			      NULL, pidgains);
}

/*! \fn int ml_SetGainVecSingle(ml_device_handle_t dev_hdl,
  ml_gainset_type_t gainset_type,
  ml_axis_index_t axis,
  ml_gain_type_t gaintype,
  ml_gain_t value)
  \brief Assigns a new gain value on a given axis for a given gain type.

  ml_SetGainVecSingle is used to modify a single gain value in the PIDF controller.
  This function takes five arguments: The first argument \a dev_hdl is the device handle of the
  MLHI controller to which this command is directed.  The second argument, \a gainset_type, 
  indicates which of the four gain sets will be changed.  The third argument \a axis denotes
  which axis's gain value is to be changed; The fourth argument \a gaintype denotes which
  gain component (P, I, D, or F) is to be changed for the given axis; The fifth argument
  \a value is the new gain value for the given axis and gain component.

  For translational axes, units are 
  <i> P(N/m), I(N/ms), D(Ns/m), F(N) </i>. 
  For rotational axes, units are 
  <i> P(Nm/rad), I(Nm/rads), D(Nms/rad), F(Nm). </i>

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If any error occurs (this function returns something besides \c ML_STATUS_OK),
  then the gain value will not have been changed.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param gainset_type The gainset to be used.
  \param axis The axis for which the new value applies.
  \param gaintype The type of gain on the given axis to replace with the new value
  \param value The new gain value to be used.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetGainVecSingle(), \c ml_SetGainVecAxis(), \c ml_GetGainVecAxis(), \c ml_SetGainVecAxes(), 
  \c ml_GetGainVecAxes(), \c ml_ResetGainVecAxes()
  
*/
int ml_SetGainVecSingle(ml_device_handle_t dev_hdl,
			ml_gainset_type_t gainset_type,
			ml_axis_index_t axis,
			ml_gain_type_t gaintype,
			ml_gain_t value)
{
  ml_set_gain_vec_single_t args_in;

  if (axis < ML_AXIS_X || axis > ML_AXIS_Z_ROT)
    return ML_STATUS_DATA_AXIS_INVALID;
  
  if (gaintype < ML_GAIN_TYPE_P || gaintype > ML_GAIN_TYPE_D)
    return ML_STATUS_DATA_AXIS_INVALID;
  
  args_in.axis = axis;
  args_in.gaintype = gaintype;
  args_in.gainset_type = gainset_type;
  args_in.value = value;
  
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_SETGAINVECSINGLE,
			      &args_in, NULL);
}

/*! \fn int ml_GetGainVecSingle(ml_device_handle_t dev_hdl,
  ml_gainset_type_t gainset_type,
  ml_axis_index_t axis,
  ml_gain_type_t gaintype,
  ml_gain_t * value)

  \brief Gets a gain value on a given axis for a given gain type.

  ml_GetGainVecSingle is used to read a single gain value when using the PIDF
  controller. This function takes 5 arguments: The first argument \a dev_hdl is the device handle of the
  MLHI controller to which this command is directed.
  The second argument, \a gainset_type, indicates which of the four gain sets will be used.
  The third argument \a axis is the axis to be read; The fourth argument \a gaintype is
  the gain component (P, I, D, or F) to be read; The fifth
  argument \a value is a pointer to where the read value is to be written.

  For translational axes, units are 
  <i> P(N/m), I(N/ms), D(Ns/m), F(N) </i>. 
  For rotational axes, units are 
  <i> P(Nm/rad), I(Nm/rads), D(Nms/rad), F(Nm). </i>

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param gainset_type The gainset to be used.
  \param axis The axis from which the value will be read.
  \param gaintype The type of the gain on the given axis to be read.
  \param value The address into which to place the read value.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetGainVecSingle(), \c ml_SetGainVecAxis(), \c ml_GetGainVecAxis(), \c ml_SetGainVecAxes(), 
  \c ml_GetGainVecAxes(), \c ml_ResetGainVecAxes()
*/

int ml_GetGainVecSingle(ml_device_handle_t dev_hdl,
			ml_gainset_type_t gainset_type,
			ml_axis_index_t axis,
			ml_gain_type_t gaintype,
			ml_gain_t * value)
{
  ml_get_gain_vec_single_t args_in;

  if (axis < ML_AXIS_X || axis > ML_AXIS_Z_ROT)
    return ML_STATUS_DATA_AXIS_INVALID;
  
  if (gaintype < ML_GAIN_TYPE_P || gaintype > ML_GAIN_TYPE_D)
    return ML_STATUS_DATA_AXIS_INVALID;
  
  args_in.axis = axis;
  args_in.gaintype = gaintype;
  args_in.gainset_type = gainset_type;
    
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETGAINVECSINGLE,
			      &args_in, value);
}

/*! \fn int ml_FindGravity(ml_device_handle_t dev_hdl, ml_forces_t* gravity)
  \brief Finds the feedforward forces/torques to cancel gravity.

  ml_FindGravity finds the feedforward forces and torques to cancel gravitational forces and torques.
  This function takes two arguments. The first argument \a dev_hdl is the device handle
  of the MLHI controller at which this command is directed. The second argument
  \a gravity is a pointer to where the discovered feedforward forces and torques will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param gravity The address into which the discovered feedforward forces and torques will be written.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection, or \n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.
  \warning This function blocks while the MLHI controller and the
  flotor attempt to discover the gravity vector. This function should probably not be
  used in the middle of a haptic simulation because it results in a short lapse of
  the client's control of the flotor. Touching the flotor while ml_FindGravity is running may result 
  in inaccurate feedforward forces and torques.

  \sa \c ml_DefyGravity(), \c ml_SetGravity(), \c ml_GetGravity()
*/

int ml_FindGravity(ml_device_handle_t dev_hdl,
		   ml_forces_t* gravity)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_FINDGRAVITY,
			      NULL, gravity);
}

/*! \fn int ml_GetGravity(ml_device_handle_t dev_hdl, ml_forces_t* read_gravity)
  \brief Reads the gravity vector data currently being stored by the MLHI.

  ml_GetGravity reads the gravity vector currently being used by the MLHI controller. 
  This function takes two arguments. The first argument \a dev_hdl is the device handle
  of the MLHI controller at which this command is directed. The second argument \a gravity
  is the address into which the read gravity vector will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param read_gravity The address into which the current gravity vector will be written.
  \return \c ML_STATUS_OK upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n

  \sa \c ml_SetGravity(), \c ml_DefyGravity(), \c ml_FindGravity()
*/

int ml_GetGravity(ml_device_handle_t dev_hdl,
		  ml_forces_t* read_gravity)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETGRAVITY,
			      NULL, read_gravity);
}

/*! \fn int ml_SetGravity(ml_device_handle_t dev_hdl, ml_forces_t gravity)
  \brief Sets a new gravity vector to the MLHI controller.

  ml_SetGravity is used to define a gravity vector in the current user coordinate
  frame. This function takes two arguments. The first argument \a dev_hdl is the device handle
  of the MLHI controller at which this command is directed. The second argument
  \a gravity is a pointer to a 6-vector to use as the new gravity vector.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If an error occurs (this function returns something other than \c ML_STATUS_OK),
  then the gravity vector will not have been changed.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param gravity The new gravity vector
  \return \c ML_STATUS_OK ( 0 )  upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetGravity(), \c ml_DefyGravity(), \c ml_FindGravity()
*/

int ml_SetGravity(ml_device_handle_t dev_hdl,
		  ml_forces_t gravity)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_SETGRAVITY,
			      &gravity, NULL);
}

/*! \fn int ml_DefyGravity(ml_device_handle_t dev_hdl)
  \brief Causes the MLHI to set feedforward forces and torques for the Normal gainset
  equal to the previously stored gravity vector.

  ml_DefyGravity causes the MLHI controller to set feedforward forces and torques 
  for the Normal gainset equal to the previously stored gravity vector. 
  This function takes one argument \a dev_hdl, which is the device handle of 
  the MLHI controller at which this command is directed.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  Proper use:
  Use the \c ml_FindGravity function to have the MLHI calculate the actual current
  gravity vector, and then use \c ml_SetGravity to set the gravity vector
  \c ml_FindGravity returned, and finally call ml_DefyGravity to begin resisting the
  correct gravity force.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_SetGravity(), \c ml_GetGravity(), \c ml_FindGravity()
*/

int ml_DefyGravity(ml_device_handle_t dev_hdl)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_DEFYGRAVITY,
			      NULL, NULL);
}


/*! \fn int ml_GetFrame(ml_device_handle_t dev_hdl, ml_position_t* frame)
  \brief Gives the offset of the current user frame from the default position.
  \todo Not Yet Implemented.

  ml_GetFrame reads the current user frame offset value and places the
  information at a given address. This function takes two arguments. The first
  argument \a dev_hdl is the device handle of the MLHI controller at which this command
  is directed. The second argument \a frame is a pointer to where the read value
  should be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param frame A pointer to an addresss into which the user frame offset can be written as a
  6-element array of ml_coord_t values [<i>x(m), y(m), z(m), roll(rad), pitch(rad), yaw(rad)</i>]
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetFrame()
*/

int ml_GetFrame(ml_device_handle_t dev_hdl,
		ml_position_t* frame)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETFRAME,
			      NULL, frame);
}

/*! \fn int ml_SetFrame( ml_device_handle_t dev_hdl, ml_position_t frame)
  \brief Sets the user coordinate frame by the given offset from default.
  \todo Not Yet Implemented.

  ml_SetFrame sets the user coordinate frame by applying a 6-vector
  offset to the default frame. This function takes two arguments. The first argument \a dev_hdl is
  the device handle of the MLHI controller at which this command is directed. The second
  argument \a frame is the 6-vector offset to apply to the user frame.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  NOTE: \c ml_SetFrame changes few variables other than frame of reference itself. 
  Desired Position and Feedforward Forces are fixed in the original frame, 
  meaning their values would differ after calling \c ml_SetFrame. 
  They are fixed to ensure stability of the flotor control while the frame transformation occurs.

  If this function returns an error, then the coordinate frame is not changed.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param frame The offset to apply to the given coordinate frame as a
  6-element array of ml_coord_t values [<i>x(m), y(m), z(m), roll(rad), pitch(rad), yaw(rad)</i>]
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetFrame()
*/

int ml_SetFrame(ml_device_handle_t dev_hdl,
		ml_position_t frame)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_SETFRAME,
			      &frame, NULL);
}

/*! \fn int ml_LockAxis(ml_device_handle_t dev_hdl, ml_axis_index_t axis)
  \brief Locks motion on a given axis in its current position using the LOCKED gainset.

  ml_LockAxis is used to lock movement of a particular axis in its current position.
  This function takes two arguments. The first argument \a dev_hdl is the device handle of the
  MLHI controller at which this command is directed. The second argument \a axis is the
  axis to be locked. If the given axis is already locked, then this function
  will have no effect.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If an error occurs (this function returns something besides \c ML_STATUS_OK), then
  no axes will have been locked.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param axis The axis whose motion will be locked.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_UnlockAxis(), \c ml_ConstrainAxis(), \c ml_GetAxisModes()
*/

int ml_LockAxis(ml_device_handle_t dev_hdl,
		ml_axis_index_t axis)
{
  if (axis < ML_AXIS_X || axis > ML_AXIS_Z_ROT)
    return ML_STATUS_DATA_AXIS_INVALID;
  
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_LOCKAXIS,
			      &axis, NULL);
}

/*! \fn int ml_UnlockAxis(ml_device_handle_t dev_hdl, ml_axis_index_t axis)
  \brief Set axis mode to normal (neither locked nor constrained) on the requested axis.  Does nothing if requested axis is already unlocked.

  ml_UnlockAxis() restores a previously constrained or locked axis to
  normal axis mode (that is, neither locked nor constrained).  This
  function takes two arguments. The first argument \a dev_hdl is the
  device handle of the MLHI controller at which this command is
  directed. The second argument \a axis designates which axis is to be
  unlocked. If this function is called for an axis which is not
  currently either locked or constrained, then the axis mode remains unchanged.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  Note: To change the constraints applied to an axis (but still constrain that axis in
  some manner), simply call \c ml_Constrain axis again with the new constraint values.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param axis The axis to be unlocked - that is, whose axis mode will be set to normal.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_LockAxis(), \c ml_ConstrainAxis(), \c ml_GetAxisModes()
*/

int ml_UnlockAxis(ml_device_handle_t dev_hdl,
		  ml_axis_index_t axis)
{
  if (axis < ML_AXIS_X || axis > ML_AXIS_Z_ROT)
    return ML_STATUS_DATA_AXIS_INVALID;
  
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_UNLOCKAXIS,
			      &axis, NULL);
}

/*! \fn int ml_ConstrainAxis(ml_device_handle_t dev_hdl, ml_axis_index_t axis, ml_coord_t minpos, ml_coord_t maxpos)
  \brief Constrains motion on a given axis between two given positions. 

  ml_ConstrainAxis is used to constrain movement on a given axis between two
  given positions. This function takes four arguments. The first argument \a dev_hdl is the device handle
  of the MLHI controller at which this command is directed. The second argument \a axis
  is the axis that the client wishes to constrain. The third argument is \a minpos, the lower
  boundary position. The fourth argument is \a maxpos, the upper boundary.position.

  Units for \a maxpos and \a minpos are <i>meters</i> for translational axes, 
  and <i>radians</i> for rotational axes.

  When an axis is constrained, the MLHI device will resist the flotor moving outside
  of the given position boundaries. If a command is given which would instruct the flotor
  to go to a position outside of the boundaries of a constrained axis, then the
  desired position of the flotor will be clipped to the closest boundary value.

  If the current position of the flotor on that axis is not between the two specified values,
  it will be moved to the nearest point within the specified boundary values.

  It is possible to call this function on an axis that is already constrained to
  adjust the constraint values. To unconstrain an axis, use the function
  \c ml_UnlockAxis.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If any error occurs (this function returns something besides \c ML_STATUS_OK, then
  no changes to axis constraints will have been made.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param axis The axis whose motion will be constrained.
  \param minpos The lower bound of the motion constraint for the given axis in meters or radians.
  \param maxpos The upper bound of the motion constraint for the given axis in meters or radians.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_LockAxis(), \c ml_UnlockAxis(), \c ml_GetAxisModes()
*/

int ml_ConstrainAxis(ml_device_handle_t dev_hdl,
		     ml_axis_index_t axis,
		     ml_coord_t minpos,
		     ml_coord_t maxpos)
{
  ml_constrain_axis_t args_in;
  args_in.axis = axis;
  args_in.minpos = minpos;
  args_in.maxpos = maxpos;

  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_CONSTRAINAXIS,
			      &args_in, NULL);
}

/*! \fn int ml_GetAxisModes(ml_device_handle_t dev_hdl,
  ml_axis_mode_vec_t * axis_mode_vec)

  \brief Returns the current axis mode (Normal, Locked or Constrained) for each of the six flotor axes.

  ml_GetAxisModes is useful in test programs to verify that the axis
  modes have been set to the requested value.  This function takes two
  arguments. The first argument \a dev_hdl is the device handle of the MLHI
  controller at which this command is directed. The second argument \a
  axis_mode_vec is a pointer to a vector capable of holding the six
  axis mode values.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param axis_mode_vec The pointer to a vector of axis_modes.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_LockAxis(), \c ml_UnlockAxis(), \c ml_ConstrainAxis()
*/

int ml_GetAxisModes(ml_device_handle_t dev_hdl,
		    ml_axis_mode_vec_t * axis_mode_vec)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_CONSTRAINAXIS,
			      NULL, axis_mode_vec);
}


/*! \fn int ml_GetBoundaryRadius(ml_device_handle_t dev_hdl, float * radius)
  \brief Gets the current radius of the bounding sphere, in meters.

  ml_GetBoundaryRadius returns the current radius of the bounding sphere in meters. 
  If the flotor's position exceeds the spherical boundary, the MLHI contoller tries to 
  push the flotor back to the nearest point on the spherical surface using the BOUNDARY gainset. 
  These forces are added to the forces computed from NORMAL or LOCK gainset.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param radius Pointer to a float where the radius value will be stored in meters.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_SetBoundaryRadius()
*/

int ml_GetBoundaryRadius(ml_device_handle_t dev_hdl,
			 float * radius)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_GETBOUNDARYRADIUS,
			      NULL, radius);
}

/*! \fn int ml_SetBoundaryRadius(ml_device_handle_t dev_hdl, float radius)
  \brief Sets the current radius of the bounding sphere, in meters.

  ml_SetBoundaryRadius sets the new value of the radius of the bounding sphere, in meters. 
  If the flotor's position exceeds the spherical boundary, the MLHI contoller tries to 
  push the flotor back to the nearest point on the spherical surface using the BOUNDARY gainset.  
  These forces are added to the forces computed from NORMAL or LOCK gainset.
  
  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param radius A float containing the new radius value in meters.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetBoundaryRadius()
*/

int ml_SetBoundaryRadius(ml_device_handle_t dev_hdl,
			 float radius)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_SETBOUNDARYRADIUS,
			      &radius, NULL);
}

/**  @}  */

/** \defgroup Interfuncs Haptic Interaction Functions
    \brief The main workhorse functions to enable haptic intereactions.

    - \c ml_GetActualPosition() and \c ml_SetDesiredPosition() functions are used to obtain the current state of the flotor.
    - \c ml_*Forces() and \c ml_*ForceAxis() functions are used to manipulate the forces applied to the flotor.
    - \c ml_GetButtonStatus() is used to obtain the state of the buttons on the handle.
    - \c ml_SetServoFrequency() and \c ml_GetServoFrequency() functions are used to manipulate the servo furequency of the
    MLHI controller.
    .

    @{
*/

/*! \fn int ml_GetActualPosition(ml_device_handle_t dev_hdl, ml_position_t* read_position)
  \brief Reads the current flotor position/orientation and places it into a given 6-vector.

  ml_GetActualPosition is used to read the current position/orientation of the flotor into a given
  address. This function takes two arguments. The first argument \a dev_hdl is the
  device handle of the MLHI controller at which this command is directed. The second
  argument \a read_position is the address into which the current position/orientation data will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param read_position A pointer to a 6-vector into which the read position/orientation will be
  placed in units of [<i>x(m), y(m), z(m), roll(rad), pitch(rad), yaw(rad)</i>]
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetDesiredPosition(), \c ml_GetVelocity()
*/

int ml_GetActualPosition(ml_device_handle_t dev_hdl,
			 ml_position_t * read_position)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETACTUALPOSITION,
			      NULL, read_position);
}

/*! \fn int ml_GetVelocity(ml_device_handle_t dev_hdl, ml_velocities_t* read_velocity)
  \brief Reads the velocity of the flotor

  ml_GetVelocity is used to read the current velocity of the flotor as a 6-vector
  and place it into a given address. This function takes two arguments. The first argument
  \a dev_hdl is the device handle of the MLHI controller at which this command is directed.
  The second argument \a read_velocity is the address into which to place the velocity
  value that is read.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param read_velocity A pointer to a 6-vector into which the velocity will be written in units of
  [<i>x(m/s), y(m/s), z(m/s), roll(rad/s), pitch(rad/s), yaw(rad/s)</i>]
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetVelocity(), \c ml_GetActualPosition()
*/

int ml_GetVelocity(ml_device_handle_t dev_hdl,
		   ml_velocities_t* read_velocity)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETVELOCITY,
			      NULL, read_velocity);
}




/*! \fn int ml_GetForces(ml_device_handle_t dev_hdl, ml_forces_t* wrench)
  \brief Reads the wrench currently being applied to the six degrees of freedom.

  ml_GetForces reads the forces currently being applied to the six
  axes of the flotor (<i>x(N), y(N), z(N), roll(Nm), pitch(Nm), yaw(Nm)</i>). 
  This function takes two arguments. 
  The first argument \a dev_hdl is the device handle of the MLHI controller
  at which this command is directed. The second argument \a wrench is the address
  into which the read force values will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param wrench A pointer to a 6-vector into which the applied wrench will be written.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetForces(), \c ml_GetForceAxis(), \c ml_SetForceAxis()
*/

int ml_GetForces(ml_device_handle_t dev_hdl,
		 ml_forces_t* wrench)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETFORCES,
			      NULL, wrench);
}

/*! \fn int ml_SetForces(ml_device_handle_t dev_hdl, ml_forces_t wrench)
  \brief Applies a given wrench for each of the 6 degrees of freedom.

  ml_SetForces is used to apply a force/torque to the flotor on all six axes. This
  function takes 2 arguments. The first argument \a dev_hdl is the device handle of the MLHI
  controller at which this command is directed. The second argument \a wrench is a
  pointer to where the desired wrench value, as a
  6-vector (<i>x(N), y(N), z(N), roll(Nm), pitch(Nm), yaw(Nm)</i>)

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If an error occurs (this function returns something besides \c ML_STATUS_OK, then
  the force values will not have been changed.

  NOTE: Control of the MLHI device via velocity commands ( \c ml_SetVelocity
  function), position commands ( \c ml_SetDesiredPosition function), force commands ( the
  \c ml_SetForce* family of functions), and raw current commands ( \c ml_SetCurrents),
  are mutually exclusive: Changing between these levels of control necessarily changes
  the manner in which the MLHI controller manges the device. As such, when one of these
  commands is issued to the device, it will replace or nullify previous settings relating
  to alternate levels of controlling the MLHI. These transitions in behavior may not
  always result in stable operation.

  For instance, if the client has used \c ml_SetDesiredPosition to instruct the flotor to move
  to a coordinate (which uses PIDF gains to determine the forces to apply to the flotor
  to accomplish this), and then calls \c ml_SetForces to command that a force be
  exerted on the flotor, the MLHI controller will begin bypassing the PIDF controller and
  ignoring desired position in favor of applying the specified forces to the flotor.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param wrench The force to be applied on each axis as a 6-vector (<i>x, y, z, roll, pitch, yaw</i>)
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetForces(), \c ml_GetForceAxis(), \c ml_SetForceAxis()
*/

int ml_SetForces(ml_device_handle_t dev_hdl,
		 ml_forces_t wrench)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_SETFORCES,
			      &wrench, NULL);
}

/*! \fn int ml_GetForceAxis(ml_device_handle_t dev_hdl, ml_axis_index_t axis, ml_force_t* force)
  \brief Reads a wrench component currently being applied to a specific axis.

  ml_GetForceAxis is used to read the force/torque being applied to a specific axis, and
  place that value into a given address. This function takes three arguments. The first
  argument \a dev_hdl is the device handle of the MLHI controller at which this command is
  directed. The second argument is \a axis, and it denotes the axis whose force/torque value
  is to be be read; The third argument is \a force, it is the address into which the
  read force/torque value will be written.

  For a translational axis, force is written in units of N, 
  and for a rotational axis torque is written in units of Nm.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param axis The axis whose applied force/torque will be read.
  \param force A pointer to where the read wrench for the given axis will be written.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetForceAxis(), \c ml_SetForces(), \c ml_GetForces()
*/

int ml_GetForceAxis(ml_device_handle_t dev_hdl,
		    ml_axis_index_t axis,
		    ml_force_t* force)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETFORCEAXIS,
			      &axis, force);
}

/*! \fn int ml_SetForceAxis(ml_device_handle_t dev_hdl, ml_axis_index_t axis, ml_force_t force)
  \brief Applies a given wrench to the flotor on a given degree of freedom.

  ml_SetForceAxis is used to selectively apply a force/torque to one of the six degrees
  of freedom of the MLHI (<i>x, y, z, roll, pitch, yaw</i>). This function takes three
  arguments. The first argument \a dev_hdl is the device handle of the MLHI controller at
  which this command is directed.  The second argument is \a axis, which denotes
  the axis or degree of freedom upon which the client wishes to apply the force/torque.
  The third argument is \a force, a pointer to the force/torque value to apply upon the given axis.

  For a translational axis, force is expected in units of N, 
  and for a rotational axis torque is expected in units of Nm.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If an error occurs (this function returns something besides \c ML_STATUS_OK, then
  the force/torque value will not have been changed.

  NOTE: Control of the MLHI device via velocity commands ( \c ml_SetVelocity
  function), position commands ( \c ml_SetDesiredPosition function), force/torque commands ( the
  \c ml_SetForce* family of functions), and raw current commands ( \c ml_SetCurrents),
  are mutually exclusive: Changing between these levels of control necessarily changes
  the manner in which the MLHI controller manges the device. As such, when one of these
  commands is issued to the device, it will replace or nullify previous settings relating
  to alteranate levels of controlling the MLHI. These transitions in behavior may not
  always result in stable operation.

  For instance, if the client has used \c ml_SetDesiredPosition to instruct the flotor to move
  to a coordinate (which uses PIDF gains to determine the forces to apply to the flotor
  to accomplish this), and then calls \c ml_SetForces to command that a force/torque be
  exerted on the flotor, the MLHI controller will begin bypassing the PIDF controller and
  ignoring desired position in favor of applying the specified forces to the flotor.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param axis The degree of freedom to which the given force/torque will be applied.
  \param force The force/torque which will be applied on the given degree of freedom.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetForceAxis(), \c ml_SetForces(), \c ml_GetForces()
*/

int ml_SetForceAxis(ml_device_handle_t dev_hdl,
		    ml_axis_index_t axis,
		    ml_force_t force)
{
  ml_set_force_axis_t args_in;
  args_in.axis=axis;
  args_in.force=force;

  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_SETFORCEAXIS,
			      &args_in, NULL);
}


/*! \fn int ml_GetButtonStatus(ml_device_handle_t dev_hdl, ml_button_t *button)
  \brief Reads the handle button status.

  ml_GetButtonStatus reads the buttons' status (on/off) on  MLHI controller's handle, 
  and have that value placed into a given address. This function takes
  two arguments. The first argument \a dev_hdl is the device handle
  of the MLHI controller at which this command is directed. The second argument \a button
  is the address into which the read button status will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param button The address into which the current button status will be written.
  \return \c ML_STATUS_OK upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
*/

int ml_GetButtonStatus(ml_device_handle_t dev_hdl, ml_button_t *button)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETBUTTONSTATUS,
			      NULL, button);
}


/*! \fn int ml_GetServoFrequency(ml_device_handle_t dev_hdl,
  float * servo_frequency)
  \brief  Gets the prevailing servo frequency in Hz.

  ml_GetServoFrequency() is used to obtain the servo frequency currently being used to control the device 
  and to invoke the tick callbacks if registered.

  The first argument, \a dev_hdl is the device handle
  of the MLHI controller to which this command is directed.

  If the given \a dev_hdl is invalid, then the function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE and no values will be read.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param servo_frequency the servo frequency being used in units of Hz.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n

  \sa \c ml_SetServoFrequency()
*/

int ml_GetServoFrequency(ml_device_handle_t dev_hdl,
			 float * servo_frequency)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_GETSERVOFREQUENCY,
			      NULL, servo_frequency);
}

/*! \fn int ml_SetServoFrequency(ml_device_handle_t dev_hdl,
  float servo_frequency)

  \brief  Sets the servo frequency in Hz.

  ml_SetServoFrequency() is used to change the servo frequency to something other
  than the default inerval of 1 KHz rate (1 mS period).

  The first argument, \a dev_hdl is the device handle
  of the MLHI controller to which this command is directed.

  The second argument, \a servo_frequency defines the frequency at which to control the device 
  and to invoke the tick callbacks if registered. 

  If the given \a dev_hdl is invalid, then the function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE and no values will be read.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param servo_frequency the desired servo frequency in units of Hz.  The valid range
  of the servo frequency is from 100 Hz to 10 KHz.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_DATA_TIME_OUT_OF_RANGE if the given \a servo_frequency does not meet the
  range criterion, ( 100.0 <= \a servo_frequency <= 10000.0 )\n

  \sa \c ml_GetServoFrequency()
*/

int ml_SetServoFrequency(ml_device_handle_t dev_hdl,
			 float servo_frequency)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_SETSERVOFREQUENCY,
			      &servo_frequency, NULL);
}


/**
   @}
*/


/** \defgroup Callbacks Callback Control Functions

    \brief These callback control functions enable the client to specify
    client-side actions to be taken in response to the occurrence of
    specific events on the controller.

    These callback functions take a pointer to a function as an argument. When the
    event associated with the callback function occurs, the given function will
    be called. Multiple calls to a particular \c RegisterCallback function overwrite
    the function which will be called for the associated event.

    The first argument given to all callback functions is the handle of
    the MLHI device which triggered the callback. Additional arguments specific to
    each callback are described below.

    \c ml_RegisterCallbackOvertemp() specifies an action to take when the flotor
    coils are believed to be overheating. The default action taken by the haptics
    device if none is specified here is to initiate an emergency shutdown. The given
    function will be called with the following arguments: (ml_device_handle_t dev_hdl, ml_coil* coil_temps),
    where coil_temps is a 6 element array containing the estimated temperatures of each of the
    6 coils.

    \c ml_RegisterCallbackFault() specifies an action to take if the
    haptics hardware detects a fault condition. The default action
    taken here if no client action is specified is to print an error notification and
    then close the connection to that device. The given function will be called with
    the following arguments: (ml_device_handle_t dev_hdl, int mlerror_value), where mlerror_value is
    a \c MLERROR code attempting to convey as best it can the reason for the shutdown.


    @{
*/

/*! \fn int ml_RegisterCallbackButtonPressed( ml_device_handle_t dev_hdl,
  button_state_callback_func_t buttonStateCallbackFunc)

  \brief Assigns a client defined function to be called if a change in button state
  occurs

  ml_RegisterCallbackButtonPressed is used to define a function to be called whenever
  the button state changes (a button which was up is now pressed down, or a previously
  pressed button was released).

  The callback function will be given two arguments.
  The first argument, \a dev_hdl, is the \c ml_device_handle_t of the MLHI device whose
  button state change has triggered the callback function. The second argument, of
  type \c ml_button_t, is a bit mask where each bit corresponds to the
  status of a button on the MLHI device (0 = released, 1 = pressed).

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The MLHI device whose button state change callback function is
  being set
  \param buttonStateCallbackFunc A function pointer to the function to be called
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is invalid.

  \sa \c ml_UnregisterCallbackButtonPressed()
*/

int ml_RegisterCallbackButtonPressed(ml_device_handle_t dev_hdl,
				     button_state_callback_func_t buttonStateCallbackFunc)
{
  return _ml_RegisterCallbackWithServer(dev_hdl,
					ML_FUNCTION_ID_REGISTERCALLBACKBUTTONPRESSED,
					(generic_func_t) buttonStateCallbackFunc);
}

/*! \fn int ml_RegisterCallbackFlotorBoundaryViolation(ml_device_handle_t dev_hdl,
  boundary_callback_func_t boundaryCallbackFunc)

  \brief Assigns a client defined function to be called if the flotor position crosses 
  the spherical boundary and/or constrained boundaries.

  ml_RegisterCallbackFlotorBoundaryViolation is used to define a function to be called
  whenever the flotor crosses the spherical boundary and/or constrained axis boundaries.

  The callback function receives two arguments.
  The first argument, \a dev_hdl, is the \c ml_device_handle_t of the MLHI device whose
  boundary violation has triggered the callback function. The second argument is a
  pointer to a 7-element \c ml_boundary_violation_t. The first 6 elements of ml_boundary_violation_t
  represents the constrained axis boundaries violations (<i>x, y, z, roll, pitch, yaw</i>), 
  -1 indicating violation of the minimum boundary, +1 indicating violation of the maximum boundary and 
  0 indicating no violation. The 7th element indicates violation of the spherical boundary, +1 indicating 
  violation and 0 indicating no violation. 

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The MLHI device whose button state change callback function is
  being set.
  \param boundaryCallbackFunc A function pointer to the function to be called.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is invalid.

  \sa \c ml_UnregisterCallbackFlotorBoundaryViolation()
*/

int ml_RegisterCallbackFlotorBoundaryViolation(ml_device_handle_t dev_hdl,
					       boundary_callback_func_t boundaryCallbackFunc)
{
  return _ml_RegisterCallbackWithServer(dev_hdl,
					ML_FUNCTION_ID_REGISTERCALLBACKFLOTORBOUNDARYVIOLATION,
					(generic_func_t) boundaryCallbackFunc);
}

/*! \fn int ml_RegisterCallbackOvertemp( ml_device_handle_t dev_hdl,
  coil_overtemp_callback_func_t coilOvertempCallbackFunc)
  \brief Assigns a client defined function to be called if the one or more flotor coils become too hot.

  ml_RegisterCallbackOvertemp is used to define a function to be called whenever the
  temperature of one or more of the coils on the flotor exceeds the hardcoded threshold.

  The callback function will be given two arguments.
  The first argument, \a dev_hdl, is the \c ml_device_handle_t of the MLHI device whose
  suspected overtemperature has triggered the callback function.
  The second argument is a 6-element array of \a ml_coil values. This argument
  contains the estimated temperatures of the six coils.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The MLHI device whose button state change callback function is
  being set
  \param coilOvertempCallbackFunc A function pointer to the function to be called
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is invalid.

  \sa \c ml_UnregisterCallbackOvertemp()
*/

int ml_RegisterCallbackOvertemp(ml_device_handle_t dev_hdl,
				coil_overtemp_callback_func_t coilOvertempCallbackFunc)
{
  return _ml_RegisterCallbackWithServer(dev_hdl,
					ML_FUNCTION_ID_REGISTERCALLBACKOVERTEMP,
					(generic_func_t) coilOvertempCallbackFunc);
}

/*! \fn int ml_RegisterCallbackFault(ml_device_handle_t dev_hdl,
    fault_callback_func_t faultCallbackFunc)

  \brief Assigns a client defined function to be called if the MLHI controller
  detects a fault condition.

  ml_RegisterCallbackFault is used to define a function to be called if the
  controller fault status changes.  A change can be either [ready -> fault]
  or [fault -> ready].

  The callback function will be given two arguments.
  The first argument, \a dev_hdl, is the \c ml_device_handle_t of the MLHI device which
  has reported the emergency shutdown.
  The second argument is of type \c ml_fault_type_t, and describes the nature of the fault. This
  code represents a best-effort attempt to describe the problem which caused the MLHI contoller 
  to stop applying currents to the flotor coils.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The MLHI device whose button state change callback function is
  being set
  \param faultCallbackFunc A function pointer to the function to be called
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is invalid.

  \sa \c ml_UnregisterCallbackFault()
*/

int ml_RegisterCallbackFault(ml_device_handle_t dev_hdl,
			     fault_callback_func_t faultCallbackFunc)
{
  return _ml_RegisterCallbackWithServer(dev_hdl,
					ML_FUNCTION_ID_REGISTERCALLBACKFAULT,
					(generic_func_t) faultCallbackFunc);
}

/*! \fn int ml_RegisterCallbackTick( ml_device_handle_t dev_hdl,
    tick_callback_func_t tickCallbackFunc)

  \brief Assigns a client defined function to be called at the top of the
  controller servo loop, immediately following the computation of the
  current flotor position.

  Tick callbacks are invoked at the frequency of the servo loop set by \a ml_SetServoFrequency().

  ml_RegisterCallbackTick is used to define a function to be called at the
  top of every servo interval.

  The callback function will be given two arguments.
  The first argument, \a dev_hdl, is the \c ml_device_handle_t of an MLHI device on
  which the tick callback was registered.
  The second argument is of type \c ml_position_t, and gives the present (just computed)
  position/orientation of the flotor.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The MLHI device whose servo interval just started.
  \param tickCallbackFunc A function pointer to the function to be called.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is invalid.

  \sa \c ml_UnregisterCallbackTick
*/

int ml_RegisterCallbackTick(ml_device_handle_t dev_hdl,
			    tick_callback_func_t tickCallbackFunc)
{
  return _ml_RegisterCallbackWithServer(dev_hdl,
					ML_FUNCTION_ID_REGISTERCALLBACKTICK,
					(generic_func_t) tickCallbackFunc);
}

/* UNREGISTER */

/*! \fn int ml_UnregisterCallbackButtonPressed(ml_device_handle_t dev_hdl)
  \brief Commands the MLHI device to cease calling (unregister) the previously registered
  callback function on occurence of the button state change event.

  The callback function needs a single argument, \a dev_hdl - the \c ml_device_handle_t of the
  MLHI device to be commanded.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The MLHI device from which button state change event callbacks
  are no longer desired.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is invalid.

  \sa \c ml_RegisterCallbackButtonPressed()
*/

int ml_UnregisterCallbackButtonPressed(ml_device_handle_t dev_hdl)
{
  return _ml_UnregisterCallbackWithServer(dev_hdl,
					  ML_FUNCTION_ID_UNREGISTERCALLBACKBUTTONPRESSED);
}

/*! \fn int ml_UnregisterCallbackFlotorBoundaryViolation(ml_device_handle_t dev_hdl)
  \brief Commands the MLHI device to cease calling (unregister) the previously registered
  callback function on occurence of the boundary violation event.

  The callback function needs a single argument, \a dev_hdl - the \c ml_device_handle_t of the
  MLHI device to be commanded.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The MLHI device from which boundary violation event callbacks
  are no longer desired.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is invalid.

  \sa \c ml_RegisterCallbackFlotorBoundaryViolation()
*/

int ml_UnregisterCallbackFlotorBoundaryViolation(ml_device_handle_t dev_hdl)
{
  return _ml_UnregisterCallbackWithServer(dev_hdl,
					  ML_FUNCTION_ID_UNREGISTERCALLBACKFLOTORBOUNDARYVIOLATION);
}

/*! \fn int ml_UnregisterCallbackOvertemp(ml_device_handle_t dev_hdl)
  \brief Commands the MLHI device to cease calling (unregister) the previously registered
  callback function on occurence of the coil overtemp event.

  The callback function needs a single argument, \a dev_hdl - the \c ml_device_handle_t of the
  MLHI device to be commanded.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The MLHI device from which coil overtemp event callbacks
  are no longer desired.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is invalid.

  \sa \c ml_RegisterCallbackOvertemp()
*/

int ml_UnregisterCallbackOvertemp(ml_device_handle_t dev_hdl)
{
  return _ml_UnregisterCallbackWithServer(dev_hdl,
					  ML_FUNCTION_ID_UNREGISTERCALLBACKOVERTEMP);
}

/*! \fn int ml_UnregisterCallbackFault( ml_device_handle_t dev_hdl)
  \brief Commands the MLHI device to cease calling (unregister) the previously registered
  callback function on occurence of the fault state change event.

  The callback function needs a single argument, \a dev_hdl - the \c ml_device_handle_t of the
  MLHI device to be commanded.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The MLHI device from fault state change event callbacks
  are no longer desired.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is invalid.

 \sa \c ml_RegisterCallbackFault()
*/

int ml_UnregisterCallbackFault(ml_device_handle_t dev_hdl)
{
  return _ml_UnregisterCallbackWithServer(dev_hdl,
					  ML_FUNCTION_ID_UNREGISTERCALLBACKFAULT);
}

/*! \fn int ml_UnregisterCallbackTick( ml_device_handle_t dev_hdl)
  \brief Commands the MLHI device to cease calling (unregister) the previously registered
  callback function on occurence of the tick event.

  The callback function needs a single argument, \a dev_hdl - the \c ml_device_handle_t of the
  MLHI device to be commanded.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The MLHI device from tick event callbacks
  are no longer desired.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is invalid.

  \sa \c ml_RegisterCallbackTick()
*/

int ml_UnregisterCallbackTick(ml_device_handle_t dev_hdl)
{
  return _ml_UnregisterCallbackWithServer(dev_hdl,
					  ML_FUNCTION_ID_UNREGISTERCALLBACKTICK);
}

/**
   @}
*/

/** \defgroup Misc Miscellaneous Functions
    \brief Functions for managing and monitoring the haptic hardware.

    These utility functions are used to monitor status of and manage the haptics
    hardware.

    The \c ml_GetTemp() function returns the estimated temperature of each of the 6
    coils. The \c ml_SetFlotorEdgeBuffer() function is used to set the maximum allowed
    boundaries of flotor movement. Finally, the \c ml_EmergencyStop() function causes
    the hardware to immediately power down.

    @{
*/


/*! \fn int ml_Connect( ml_device_handle_t* dev_hdl, char* addr)

  \brief Creates a connection between a client application program and
  the MLHI controller, supplying a device handle for that MLHI
  controller to the client program.  A client program must call
  ml_Connect() first, before any other calls, in order to obtain a
  device handle, which will be passed to all subsequent MLHI API
  calls.

  \param dev_hdl A local device handle referring to the MLHI controller will be written to this address.  All other MLHI API calls require such a handle as their first parameter.
  \param addr A string-valued network address - the IP address or host name of the MLHI controller.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if an error occurred obtaining a new device handle \n
  or \c ML_STATUS_COMM_CONNECTION if an error occurred during transmission.

  \sa \c ml_Disconnect()
*/

int ml_Connect(ml_device_handle_t* dev_hdl,
	       const char* const addr)
{
  int status = ML_STATUS_OK;
  
  // establish connection, obtain a socket for ml_* functions
  int socket_ml_func;
  in_addr addr_in_addr;
#ifdef WIN32
  addr_in_addr.S_un.S_addr = inet_addr(addr);
#else
  inet_aton(addr, &addr_in_addr);
#endif

  if ((socket_ml_func=connect_to_server_tcp(addr_in_addr, maglev_port)) < 0)
    status = ML_STATUS_COMM_CONNECTION;

  // allocate memory and set pointer to ml_device_t object
  if (status==ML_STATUS_OK)
    {
      *dev_hdl = (ml_device_handle_t) malloc(sizeof(struct ml_device_t));
      if (dev_hdl == NULL)
	status = -1;

      memcpy(&(**dev_hdl).server_addr, &addr_in_addr, sizeof(in_addr));
      (**dev_hdl).socket_ml_func = socket_ml_func;
      // no callback registered at this point
      (**dev_hdl).socket_ml_callback = -1;
      (**dev_hdl).button_state_callback_func = (button_state_callback_func_t) NULL;
      (**dev_hdl).boundary_callback_func = (boundary_callback_func_t) NULL;
      (**dev_hdl).coil_overtemp_callback_func = (coil_overtemp_callback_func_t) NULL;
      (**dev_hdl).fault_callback_func = (fault_callback_func_t) NULL;
      (**dev_hdl).tick_callback_func = (tick_callback_func_t) NULL;

#ifdef WIN32
	(**dev_hdl).comm_mutex = CreateMutex(NULL, FALSE, NULL);
	if ((**dev_hdl).comm_mutex==NULL)
	{
		printf("CreateMutex error: %d\n", GetLastError());
		return -1;
	}
#else
      pthread_mutex_init(&(**dev_hdl).comm_mutex, NULL);
#endif
    }

  return status;
    }

/*! \fn int ml_Disconnect(ml_device_handle_t dev_hdl)
  \brief Closes the connection between a client application and a given MLHI controller.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The ID of the device whose connection is to be closed.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if \a dev_hdl does not refer to a currently
  active connection.

  \sa \c ml_Connect()
*/

int ml_Disconnect(ml_device_handle_t dev_hdl)
{
	int return_val; 
  if (dev_hdl==NULL)
    return ML_STATUS_INVALID_DEVICE_HANDLE;

  // stop callback stuff
  _ml_StopCallback(dev_hdl);

  printf("stopped callbacks\n");

  // let the server know of disconnection
  return_val = invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
						ML_FUNCTION_ID_DISCONNECT,
					&dev_hdl->socket_ml_callback, NULL);

  printf("invoked ml_Disconnect remotely\n");

  // close socket
#ifdef WIN32
  closesocket(dev_hdl->socket_ml_func);
#else
  close(dev_hdl->socket_ml_func);
#endif
  dev_hdl->socket_ml_func = -1;

  //  free memory allocated to the device_handle
  free(dev_hdl); dev_hdl = NULL;
  
  return return_val;
}


/*! \fn int ml_GetTemp(ml_device_handle_t dev_hdl, ml_temps_t *read_temp)
  \brief Gets the estimated temperatures of the  coils in degrees Celsius.

  ml_GetTemp is used to retrieve the estimated temperatures of the flotor coils, and
  place those values into a given address. This function takes two arguments. The first
  argument \a dev_hdl is the device handle of the MLHI controller at which this command is directed.
  The second argument \a read_temp is the address into which the temperature values will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param read_temp The address into which the estimated temperature values will be written in celsius.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.
*/

int ml_GetTemp(ml_device_handle_t dev_hdl,
	       ml_temps_t *read_temp)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETTEMP,
			      NULL, read_temp);
}

/*! \fn int ml_Shutdown(ml_device_handle_t dev_hdl)
  \brief Commands a shutdown of the MLHI controller.

  ml_Shutdown sends a shutdown command to the MLHI controller, causing it to reboot.
  This function takes one argument \a dev_hdl, 
  which is the device handle of the MLHI controller at which this command is directed.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

*/

int ml_Shutdown(ml_device_handle_t dev_hdl)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_SHUTDOWN,
			      NULL, NULL);
}

/*! \fn int ml_GetVersion(ml_device_handle_t dev_hdl,
  char * client_version, char * server_version)
  \brief Gets versions of API's running on server and client.

  The first argument, \a dev_hdl is the device handle
  of the MLHI controller to which this command is directed.

  The second argument, \a client_version is the address into which 
  client API version is written to as a string.

  The second argument, \a server_version is the address into which 
  server API version is written to as a string.

  If the given \a dev_hdl is invalid, then the function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE and no values will be read.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param client_version string into thich the version of client API is written
  \param server_version string into thich the version of server API is written
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n

*/

int ml_GetVersion(ml_device_handle_t dev_hdl, char * client_version, char * server_version)
{
  strcpy(client_version, CLIENT_VERSION);
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_GETVERSION,
			      NULL, server_version);
}

/*! \fn int ml_ResetFault(ml_device_handle_t dev_hdl)
  \brief If controller is in a fault state, ml_ResetFault() will attempt
  to reset the controller to the READY state.

  ml_ResetFault() is used to clear a fault condition so that normal controller
  operation may resume.  The ml_ResetFault() call will succeed only if all of the
  prevailing fault conditions have been removed.

  The first argument, \a dev_hdl is the device handle
  of the MLHI controller to which this command is directed.

  If the given \a dev_hdl is invalid, then the function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE and no values will be read.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n

  \sa \c ml_GetFault()
*/

int ml_ResetFault(ml_device_handle_t dev_hdl)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_RESETFAULT,
			      NULL, NULL);
}


/*! \fn int ml_GetFault(ml_device_handle_t dev_hdl, ml_fault_t* fault)
  \brief Gets the current fault status of the MLHI controller.

  ml_GetFault() is used to read fault conditions of the controller.

  The first argument, \a dev_hdl is the device handle
  of the MLHI controller to which this command is directed.

  The second argument, \a fault is the fault status of the controller.
  If fault->value==0, there is no fault condition. If nonzero, 
  there exists at least one fault condition on the MLHI controller.
  For each \c ml_fault_type_t other than ML_FAULT_TYPE_CLEAR, 
  fault conditions can be checked by (fault->values & \a ml_fault_type_t=?1).
  ML_FAULT_TYPE_SENSOR_OUT_OF_RANGE indicates at least one of the three sensors are out of the valid range. 
  ML_FAULT_TYPE_COIL_OVERTEMP indicates at least one of the six coils exceed maximum temperature limit.
  ML_FAULT_TYPE_COIL_OVERCURRENT indicates at least one of the six coils experience high current for 
  a long period of time (approximately 2 A for 1 minute).

  If the given \a dev_hdl is invalid, then the function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE and no values will be read.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param fault Pointer to which the current fault status of the MLHI controller is written
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n

  \sa \c ml_ResetFault()
*/

int ml_GetFault(ml_device_handle_t dev_hdl, ml_fault_t * fault)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_GETFAULT,
			      NULL, fault);
}




/**
   @}
*/

/** \defgroup Robotic Robotic Functions
    \brief Functions for managing PIDF controller's desired position.

    These robotic functions are used to monitor status of and manage the desired position used by the MLHI controller
    to do PIDF control.

    - \c ml_GetDesiredPosition() and \c ml_SetDesiredPosition() functions are used to manipulate the desired position 
    for the MLHI controller.
    - \c ml_SetVelocity() is used to command desired velocities at which to move the flotor.

    @{
*/


/*! \fn int ml_GetDesiredPosition(ml_device_handle_t dev_hdl, ml_position_t* desired_position)
  \brief Returns the current desired flotor position, placing it into a given 6-vector

  ml_GetDesiredPosition reads the current desired position of the flotor.
  This function takes two arguments. The first argument \a dev_hdl is the
  device handle of the MLHI controller at which this command is directed. The second
  argument \a desired_position is the address into which the current desired position data will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param desired_position A pointer to a 6-vector into which the desired position 
  will be placed in units of  [<i>x(m), y(m), z(m), roll(rad), pitch(rad), yaw(rad)</i>].
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetDesiredPosition(), \c ml_GetActualPosition()
*/

int ml_GetDesiredPosition(ml_device_handle_t dev_hdl,
			  ml_position_t * desired_position)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_GETDESIREDPOSITION,
			      NULL, desired_position);
}

/*! \fn int ml_SetDesiredPosition(ml_device_handle_t dev_hdl, ml_position_t position)
  \brief Changes the goal position/orientation for the PIDF controller.

  ml_SetDesiredPosition is used to set the desired position of the flotor. This function takes two arguments.
  The first argument \a dev_hdl is the device handle of the MLHI controller at which this
  command is directed. The second argument  \a position, is the new position/orientation for the PIDF controller.

 \todo
  This function first validates whether or not the given position
  exceeds the position boundaries. If the given position exceeds the
  boundaries then the function will return \c ML_STATUS_POS_OUT_OF_RANGE.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If an error occurs (this function returns something besides \c ML_STATUS_OK, then
  the desired position value will not have been changed.

  NOTE: Control of the MLHI device via velocity commands ( \c ml_SetVelocity
  function), position commands ( \c ml_SetDesiredPosition function), force commands ( the
  \c ml_SetForce* family of functions), and raw current commands ( \c ml_SetCurrents),
  are mutually exclusive: Changing between these levels of control necessarily changes
  the manner in which the MLHI controller manges the device. As such, when one of these
  commands is issued to the device, it will replace or nullify previous settings relating
  to alternate levels of controlling the MLHI. These transitions in behavior may not
  always result in stable operation.

  For instance, if the client has used \c ml_SetDesiredPosition to instruct the flotor to move
  to a coordinate (which uses PIDF gains to determine the forces to apply to the flotor
  to accomplish this), and then calls \c ml_SetForces to command that a force be
  exerted on the flotor, the MLHI controller will begin bypassing the PIDF controller and
  ignoring desired position in favor of applying the specified forces to the flotor.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param position The coordinate of the new desired position as a 6-vector [<i>x(m), y(m), z(m), roll(rad), pitch(rad), yaw(rad)</i>]
  \return \c ML_STATUS_OK ( 0 ) upon success,\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_POS_OUT_OF_RANGE if the given \a position argument contains any
  coordinates outside of the position boundaries,\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetDesiredPosition(), \c ml_SetVelocity()
*/

int ml_SetDesiredPosition(ml_device_handle_t dev_hdl,
			  ml_position_t position)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_SETDESIREDPOSITION,
			      &position, NULL);
}


/*! \fn int ml_SetVelocity(ml_device_handle_t dev_hdl, ml_velocities_t velocity)
  \brief Applies a given velocity to the flotor.

  ml_SetVelocity is used to define a desired velocity vector for the flotor, causing
  the MLHI controller to move the flotor accordingly. This function takes two arguments.
  The first argument \a dev_hdl is the device handle of the MLHI controller at which this
  command is directed. The second argument \a velocity is a pointer to the new desired
  velocity (as a 6-vector) to be set.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If an error occurs (this function returns something besides \c ML_STATUS_OK, then
  the desired velocity value will not have been changed.

  NOTE: Control of the MLHI device via velocity commands ( \c ml_SetVelocity
  function), position commands ( \c ml_SetDesiredPosition function), force commands (the
  \c ml_SetForce* family of functions), and raw current commands ( \c ml_SetCurrents),
  are mutually exclusive: Changing between these levels of control necessarily changes
  the manner in which the MLHI controller manges the device. As such, when one of these
  commands is issued to the device, it will replace or nullify previous settings relating
  to alteranate levels of controlling the MLHI. These transitions in behavior may not
  always result in stable operation.

  For instance, if the client has used \c ml_SetDesiredPosition to instruct the flotor to move
  to a coordinate (which uses PIDF gains to determine the forces to apply to the flotor
  to accomplish this), and then calls \c ml_SetForces to command that a force be
  exerted on the flotor, the MLHI controller will begin bypassing the PIDF controller and
  ignoring desired position in favor of applying the specified forces to the flotor.
  These transitions in behavior may not always result in stable operation.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param velocity A pointer to a 6-vector of the velocity to apply in units of 
  [<i>x(m/s), y(m/s), z(m/s), roll(rad/s), pitch(rad/s), yaw(rad/s)</i>]
  \return ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetVelocity(), \c ml_SetDesiredPosition()
*/

int ml_SetVelocity(ml_device_handle_t dev_hdl,
		   ml_velocities_t velocity)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
			      ML_FUNCTION_ID_SETVELOCITY,
			      &velocity, NULL);
}

/**
   @}
*/

/** \defgroup DirectHWfuncs Low Level Functions
    \brief Functions to directly interact with the haptic hardware on a low level

    These functions are provided for the sake of completeness, though
    their general use is not reccommended. They should be used only for hardware
    debugging purposes, or, if used during standard operation, only by people who are
    familiar with the MLHI hardware, its limitations, and operation (as some of
    these functions can potentially set values which are harmful to the haptics hardware).

    - \c ml_*Currents() and \c ml_*Current() functions are used 
    to manipulate the currents applied to the coils.
    - \c ml_*CurrentLimit() and \c ml_*CurrentLimits() are used to manipulate the limits 
    on how much currents can be applied to the coils.
    - \c ml_SetSpeedLimits() and \c ml_GetSpeedLimits() are used to manipulate the maximum speed
    at which the flotor is allowed to move.
    - \c ml_SetLatCellOffset() and \c ml_GetLatCellOffset() are used 
    to manipulate the offset to sensing lat cells.
    - \c ml_GetCellPosition() is used to read the [x,y] positions of three LED light spots.
    - \c ml_GetRawCellData() is used to read the raw voltages, from which LED positions are computed, 
    for the three lat cells.
    - \c ml_SetAuxDac() is used to output voltage to the aux diagnostic output.

    Using the Direct Hardware Functions clients can directly output currents to the 6
    coils ( \c ml_SetCurrents()), and the gripper ( \c ml_SetGraspCurrent()); read the
    currents currently being applied to the six coils ( \c ml_GetCurrents()) and the
    gripper ( \c ml_GetGraspCurrent()); set the limit on the amount of current that can
    be applied to each of the coils ( \c ml_SetCurrentLimits()), and the
    gripper ( \c ml_SetGripCurrentLimit()); read the raw values currently reported by
    the position sensing LEDs ( \c ml_GetCellPosition()); apply a calibration offset
    to those LEDs ( \c ml_SetLatCellOffset()).

    @{
*/

/*! \fn int ml_GetCurrents(ml_device_handle_t dev_hdl, ml_currents_t* read_currents)
  \brief Reads the currents currently being applied through the 6 coils.

  ml_GetCurrents is used to retrieve the current being output to each of the six coils,
  and places those values in a given address. This function takes two arguments.
  The first argument \a dev_hdl is
  the device handle of the MLHI controller at which this command is directed.
  The second argument \a read_currents is a pointer to where the current values will be
  placed when they are read.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param read_currents The address into which to place the currents being applied in amps.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetCurrents(), \c ml_GetCurrentLimits(), \c ml_SetCurrentLimits()
*/

int ml_GetCurrents(ml_device_handle_t dev_hdl,
		   ml_currents_t * read_currents)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl,
				ML_FUNCTION_ID_GETCURRENTS,
				NULL, read_currents);
}

/*! \fn int ml_SetCurrents(ml_device_handle_t dev_hdl, ml_currents_t client_currents)
  \brief Directory sets currents to the six coils.

  ml_SetCurrents is used to specify directly the current to output to each of the six
  flotor coils. This function takes two arguments. The first argument \a dev_hdl is
  the device handle of the MLHI controller at which this command is directed.
  The second argument \a client_currents is a pointer to the new desired current values.

  This function first validates that the specified \a client_currents do not exceed the
  specified maximum current values (specified through the \c ml_SetCurrentLimits()
  function). Any values exceeding the specified maximum values will be clipped to the
  maximum value, and then output to the coils. If any values have been clipped, the
  function will return \c ML_STATUS_CURRENT_OUT_OF_RANGE after writing the currents.

  NOTE: Control of the MLHI device via velocity commands ( \c ml_SetVelocity
  function), position commands ( \c ml_SetDesiredPosition function), force commands ( the
  \c ml_SetForce* family of functions), and raw current commands ( \c ml_SetCurrents),
  are mutually exclusive: Changing between these levels of control necessarily changes
  the manner in which the MLHI controller manges the device. As such, when one of these
  commands is issued to the device, it will replace or nullify previous settings relating
  to alteranate levels of controlling the MLHI. These transitions in behavior may not
  always result in stable operation.

  For instance, if the client has used \c ml_SetDesiredPosition to instruct the flotor to move
  to a coordinate (which uses PIDF gains to determine the forces to apply to the flotor
  to accomplish this), and then calls \c ml_SetForces to command that a force be
  exerted on the flotor, the MLHI controller will begin bypassing the PIDF controller and
  ignoring desired position in favor of applying the specified forces to the flotor.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If an error besides \c ML_STATUS_CURRENT_OUT_OF_RANGE occurs (if this function returns
  something other than \c MLEREROR_NOERROR or \c ML_STATUS_CURRENT_OUT_OF_RANGE),
  then no current values will have been written.

  If a communication error occurs that prevents transmission of the values, and some of
  the values also exceeded their specified maximums (meaning that a return of both
  \c ML_STATUS_CURRENT_OUT_OF_RANGE and \c ML_STATUS_COMMCONNECTION would be appropriate),
  \c ML_STATUS_COMMCONNECTION will be given precedence and returned, since it is the
  more significant error (it prevented the current values, out of range as they might have
  been, from even being transmitted to the MLHI controller).

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param client_currents A pointer to the currents values to apply to each of the 6 coils in amps.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_DATA_CURRENT_OUT_OF_RANGE if one or more of the values specified at
  \a client_currents exceeded the maximum current, \n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetCurrents(), \c ml_GetCurrentLimits(), \c ml_SetCurrentLimits()
*/

int ml_SetCurrents(ml_device_handle_t dev_hdl,
		   ml_currents_t client_currents)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_SETCURRENTS,
			      &client_currents, NULL);
}

/*! \fn int ml_GetCurrentLimits(ml_device_handle_t dev_hdl, ml_currents_t* maxcurrents)
  \brief Reads the limit on the current which can be applied to each of the 6 coils.

  ml_GetCurrentLimits is used to read the current limit values, and place them into
  a given address. This function takes two arguments. The first argument \a dev_hdl is
  the device handle of the MLHI controller at which this command is directed. The
  second argument \a maxcurrents is the address into which the read current
  values will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param maxcurrents A pointer to where the maximum current values for each of the 6
  coils will be written in amps.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetCurrentLimits(), \c ml_GetCurrents(), \c ml_SetCurrents()
*/

int ml_GetCurrentLimits(ml_device_handle_t dev_hdl,
			ml_currents_t* maxcurrents)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_GETCURRENTLIMITS,
			      NULL, maxcurrents);
}

/*! \fn int ml_SetCurrentLimits(ml_device_handle_t dev_hdl, ml_currents_t maxcurrents)
  \brief Sets current limits for each of the six coils.

  ml_SetCurrentLimits is used to specify the maximum current that may be applied to
  each coil. This function takes two arguments. The first argument \a dev_hdl is
  the device handle of the MLHI controller at which this command is directed.
  The second argument \a maxcurrents contains the maximum current value for each coil.

  Whenever an attempt is made to output currents to the coils (whether it be from the
  client or from the MLHI controller), those current values are first checked to make sure
  that they do not exceed the maximums specified here. If any value exceeds the limits
  set here, then that exceeding current value is clipped down to the limit value.  (i.e.
  if a value of 3 A is output to a coil with a limit of 2 A, then a value of 2 A would be
  applied to the coil).

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If an error occurs (this function returns something other than \c ML_STATUS_OK),
  then no maximum current values will have been written.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param maxcurrents The address of the maximum current values for each of the six coils in amps.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetCurrentLimits(), \c ml_GetCurrents(), \c ml_SetCurrents()
*/

int ml_SetCurrentLimits(ml_device_handle_t dev_hdl,
			ml_currents_t maxcurrents)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_SETCURRENTLIMITS,
			      &maxcurrents, NULL);
}

/*! \fn int ml_GetGraspCurrent(ml_device_handle_t dev_hdl, ml_current_t* current)
  \brief Reads the current being applied to the grasp axis.

  ml_GetGraspCurrent is used to read the current being applied to the grasp axis, and
  place that value into a given address. This function takes two arguments. The first
  argument \a dev_hdl is the device handle of the MLHI controller at which this command is
  directed. The second argument \a current is the address into which the read current
  value will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param current The address into which the read current will be written in amps.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetGraspCurrent(), \c ml_SetGraspCurrentLimit(), \c ml_GetGraspCurrentLimit(),
*/

int ml_GetGraspCurrent(ml_device_handle_t dev_hdl,
		       ml_current_t* current)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_GETGRASPCURRENT,
			      NULL, current);
}

/*! \fn int ml_SetGraspCurrent(ml_device_handle_t dev_hdl, ml_current_t current)
  \brief Sets a current to apply to the gripper.

  ml_SetGraspCurrent is used to directly specify the current to the grasp axis.
  This function takes two arguments. The first argument \a dev_hdl is
  the device handle of the MLHI controller at which this command is directed. The second
  argument \a current is the current value to output to the grasp axis.

  This function first validates that the supplied \a current value does not exceed
  the specified maximum current value for the grasp axis (the maximum value can be
  set via the \c ml_SetGraspCurrentLimit() function). If the supplied current value
  exceeds the maximum current value, then the supplied current value will be clipped
  to the maximum value and sent to the MLHI controller. If this occurs, the function
  will return \c ML_STATUS_CURRENT_OUT_OF_RANGE.

  If a communication error occurs that prevents transmission of the current value, and
  additionally the given current value exceeded the specified maximum (meaning that a
  return of both \c ML_STATUS_CURRENT_OUT_OF_RANGE and \c ML_STATUS_COMMCONNECTION would be
  appropriate), \c ML_STATUS_COMMCONNECTION will be given precedence and returned, since it
  is the more significant error (it prevented the current value, out of range as it was,
  from even being transmitted to the MLHI controller).

  NOTE: The grasp axis operates <b>independently</b> of the six
  principal axes. Thus, it is possible to be controlling the six principal axes through
  position commands, and the grasp axis via current commands, etc.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If an error other than \c ML_STATUS_CURRENT_OUT_OF_RANGE, (this function returns
  something other than \c ML_STATUS_OK or \c ML_STATUS_CURRENT_OUT_OF_RANGE),
  then the grasp current value will not have been changed.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param current The current to apply to the gripper in amps.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_DATA_CURRENT_OUT_OF_RANGE if the given current value exceeds the
  specified maximum current for the grasp axis, \n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetGraspCurrent(), \c ml_SetGraspCurrentLimit(), \c ml_GetGraspCurrentLimit()
*/

int ml_SetGraspCurrent(ml_device_handle_t dev_hdl,
		       ml_current_t current)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_SETGRASPCURRENT,
			      &current, NULL);
}

/*! \fn int ml_GetGraspCurrentLimit(ml_device_handle_t dev_hdl, ml_current_t* maxcurrent)
  \brief Reads the current limit which can be applied to the grasp axis.

  ml_GetGraspCurrentLimit is used to read the current limit for the grasp axis. 
  This function takes two arguments. The first argument \a dev_hdl is
  the device handle of the MLHI controller at which this command is directed. The second
  argument \a maxcurrent is the address into which the read value will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param maxcurrent The address into which the current maximum will be written in amps.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetGraspCurrentLimit(), \c ml_SetGraspCurrent(), \c ml_GetGraspCurrent()
*/

int ml_GetGraspCurrentLimit(ml_device_handle_t dev_hdl,
			    ml_current_t* maxcurrent)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_GETGRASPCURRENTLIMIT,
			      NULL, maxcurrent);
}


/*! \fn int ml_SetGraspCurrentLimit(ml_device_handle_t dev_hdl, ml_current_t maxcurrent)
  \brief Sets a limit on the current which can be applied to the grasp axis.

  ml_SetGraspCurrentLimit is used to specify the maximum current that can be output
  to the grasp axis. This function takes two arguments. The first argument
  \a dev_hdl is the device handle of the MLHI controller at which this command is directed.
  The second argument \a maxcurrent is the maximum current value.

  Whenever an attempt is made to output currents (whether it be from the client or from
  the MLHI controller), those current values are first checked to make sure that they do
  not exceed the maximums specified here. If any value exceeds the limits
  set here, then that exceeding current value is clipped down to the limit value.  (i.e.
  if a value of 3 A is output to a coil with a limit of 2 A, then a value of 2 A would be
  applied to the coil).

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If an error occurs (this function returns something other than \c ML_STATUS_OK),
  then the maximum current grasp limit will not have been changed).

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param maxcurrent The value of the maximum allowed current to be applied to the
  gripper in amps.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetGraspCurrentLimit(), \c ml_SetGraspCurrent(), \c ml_GetGraspCurrent()
*/

int ml_SetGraspCurrentLimit(ml_device_handle_t dev_hdl,
			    ml_current_t maxcurrent)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_SETGRASPCURRENTLIMIT,
			      &maxcurrent, NULL);
}

/*! \fn int ml_GetLatCellLimit(ml_device_handle_t dev_hdl, float *limit)
  \brief Reads the maximum x/y data for the LED position sensor

  ml_GetLatCellLimit is used to read out the maximum x/y data for the three
  position sensing lat cells and have those values placed into
  the given address. This function takes two arguments. The first argument
  \a dev_hdl is the device handle of the MLHI controller at which this command is directed. The
  other argument, \a limit, is the address into which
  the value for lat cell sensors will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param limit The address into which the maximum lat cell output will be written in meters. 
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_GetLatCellOffset(), \c ml_SetLatCellOffset(), \c ml_GetCellPosition()
*/

int ml_GetLatCellLimit(ml_device_handle_t dev_hdl,
		       float *limit)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_GETLATCELLLIMIT,
			      NULL, limit);
}


/*! \fn int ml_GetSpeedLimits(ml_device_handle_t dev_hdl, ml_velocities_t* maxspeeds)
  \brief Reads and returns the limit on the maximum speed at which flotor can move along each axis.

  ml_GetSpeedLimits is used to read the maximum speeds, and place them into
  a given address. This function takes two arguments. The first argument \a dev_hdl is
  the device handle of the MLHI controller at which this command is directed. The
  second argument \a maxspeeds is the address into which the read current
  values will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param maxspeeds A pointer to where the maximum speeds for each of the 6 axes will be written in meters/second for translational axes and radians/second for rotational axes.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetSpeedLimits(), \c ml_GetVelocity(), \c ml_SetVelocity()
*/

int ml_GetSpeedLimits(ml_device_handle_t dev_hdl,
			ml_velocities_t* maxspeeds)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_GETSPEEDLIMITS,
			      NULL, maxspeeds);
}

/*! \fn int ml_SetSpeedLimits(ml_device_handle_t dev_hdl, ml_velocities_t maxspeeds)
  \brief Sets a limit on speeds at which the flotor is allowed to move at each of the 6 coils.

  ml_SetSpeedLimits is used to specify the maximum speeds at which the flotor is allowed to move at.
  This function takes two arguments. The first argument \a dev_hdl is
  the device handle of the MLHI controller at which this command is directed.
  The second argument \a maxspeeds contains the maximum speeds for each axis.

  When the flotor exceeds maximum speed along any axis, all forces are shut down, 
  and fault callback handler is invoked if registered ml_RegisterCallbackFault().

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If the \a maxspeeds argument contains any negative values, this function will return
  \c ML_STATUS_DATA_SPEED_OUT_OF_RANGE, and speed limits remain unchanged for all six axes.

  If an error occurs (this function returns something other than \c ML_STATUS_OK),
  then no maximum current values will have been written.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param maxspeeds The address of the maximum current values for each of the six coils in amps.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection, or \n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetSpeedLimits(), \c ml_GetVelocity(), \c ml_SetVelocity()
*/

int ml_SetSpeedLimits(ml_device_handle_t dev_hdl,
			ml_velocities_t maxspeeds)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_SETSPEEDLIMITS,
			      &maxspeeds, NULL);
}


/*! \fn int ml_SetLatCellOffset(ml_device_handle_t dev_hdl,
  ml_sensor_offset_t cella, ml_sensor_offset_t cellb, ml_sensor_offset_t cellc)
  \brief Sets a calibration offset for each of the three position sensing lat cells

  ml_SetLatCellOffset is used to set a calibration offset for the lat cells. This
  function takes four arguments. The first argument \a dev_hdl is
  the device handle of the MLHI controller at which this command is directed.
  The other three arguments \a cella, \a cellb, and \a cellc, are the offsets for
  lat cells a, b, and c, respectively.

  The lat cell calibration offsets are applied to the (x,y) sensor spot coordinates
  returned by the sensor loop. These offsets can be used to re-calibrate
  any systematic errors. This function should only be called under certain
  circumstances when a calibration fixture is in place.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  If an error occurs (this function returns something other than \c ML_STATUS_OK),
  then the lat cell offset will not have been changed.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param cella The offsest for Lat Cell A as a 3-vector [<i>x(m), y(m), t(rad)</i>]
  \param cellb The offsset for Lat Cell B as a 3-vector [<i>x(m), y(m), t(rad)</i>]
  \param cellc The offsset for Lat Cell C as a 3-vector [<i>x(m), y(m), t(rad)</i>]
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection, or \n
  \c ML_STATUS_COMM_CONNECTION if there was an error sending information.

  \sa \c ml_GetLatCellOffset(), \c ml_GetCellPosition()
*/

int ml_SetLatCellOffset(ml_device_handle_t dev_hdl,
			ml_sensor_offset_t cella,
			ml_sensor_offset_t cellb,
			ml_sensor_offset_t cellc)
{
  ml_set_lat_cell_offset_t args;  
  memcpy(args.cells, &cella, sizeof(ml_sensor_offset_t));
  memcpy(args.cells+1, &cellb, sizeof(ml_sensor_offset_t));
  memcpy(args.cells+2, &cellc, sizeof(ml_sensor_offset_t));
    
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_SETLATCELLOFFSET,
			      &args, NULL);
}

/*! \fn int ml_GetLatCellOffset(ml_device_handle_t dev_hdl,
  ml_sensor_offset_t *cella, ml_sensor_offset_t *cellb, ml_sensor_offset_t *cellc)
  \brief Reads and returns the calibration offset for each of the three position sensing
  lat cells

  ml_GetLatCellOffset is used to read out the calibration offsets currently being applied
  to each of the three position sensing lat cells and have those values placed into
  the addresses of given 3-vectors. This function takes four arguments. The first argument
  \a dev_hdl is the device handle of the MLHI controller at which this command is directed. The
  other there arguments \a cella, \a cellb, and \a cellc, are the addresses into which
  the calibration offset values for lat cell sensors a, b, and c respectively will be
  placed.

  struct member <i>t</i> is expected in radians, 
  while the other members <i>x, y</i> are expected in meters

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param cella The address into which the calibration offset for lat cell a will be written.
  \param cellb The address into which the calibration offset for lat cell b will be written.
  \param cellc The address into which the calibration offset for lat cell c will be written.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_SetLatCellOffset(), \c ml_GetCellPosition()
*/

int ml_GetLatCellOffset(ml_device_handle_t dev_hdl,
			ml_sensor_offset_t *cella,
			ml_sensor_offset_t *cellb,
			ml_sensor_offset_t *cellc)
{
  ml_set_lat_cell_offset_t args_out;
  int return_val = invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
					ML_FUNCTION_ID_GETLATCELLOFFSET,
					NULL, &args_out);
  if (return_val == ML_STATUS_OK)
    {
      memcpy(cella, args_out.cells, sizeof(ml_sensor_offset_t));
      memcpy(cellb, args_out.cells+1, sizeof(ml_sensor_offset_t));
      memcpy(cellc, args_out.cells+2, sizeof(ml_sensor_offset_t));
    }
  return return_val;
}

/*! \fn int ml_GetCellPosition(ml_device_handle_t dev_hdl,
  ml_sensor_t *cella, ml_sensor_t *cellb, ml_sensor_t *cellc)
  \brief Reads the current raw cell position values and places them into given
  addresses

  ml_GetCellPosition is used to get the dewarped (x,y) coordinate values of the
  LED light spots on the position sensors in meters. This function takes four arguments. The first argument
  \a dev_hdl is the device handle of the MLHI controller at which this command is directed.
  The other three arguments \a cella, \a cellb, and \a cellc, are pointers to three
  2-vectors into which the (x,y) coordinate values for lat cell a, lat cell b, and
  lat cell c, respectively, will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param cella The address into which the (x,y) coordinate for lat cell a will be written.
  \param cellb The address into which the (x,y) coordinate for lat cell b will be written.
  \param cellc The address into which the (x,y) coordinate for lat cell c will be written.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.

  \sa \c ml_GetLatCellOffset(), \c ml_SetLatCellOffset()
*/

int ml_GetCellPosition(ml_device_handle_t dev_hdl,
		       ml_sensor_t *cella,
		       ml_sensor_t *cellb,
		       ml_sensor_t *cellc)
{
  ml_get_cell_position_t args_out;
  int return_val = invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
					ML_FUNCTION_ID_GETCELLPOSITION,
					NULL, &args_out);

  if (return_val==ML_STATUS_OK)
    {
      memcpy(cella, args_out.cells, sizeof(ml_sensor_t));
      memcpy(cellb, args_out.cells+1, sizeof(ml_sensor_t));
      memcpy(cellc, args_out.cells+2, sizeof(ml_sensor_t));
    }

  return return_val;
}

/*! \fn int ml_GetRawCellData(ml_device_handle_t dev_hdl,
  ml_sensor_raw_data_t *sensorRaw)
  \brief Reads the raw voltages from the three lat cells.

  ml_GetRawCellData is used to get the raw sensor values produced by the
  lat cells in volts. The values are between -10 V and +10 V.

  This function takes two arguments. The first argument
  \a dev_hdl is the device handle of the MLHI controller at which this command is directed.
  The other argument, \a sensorRaw, is a pointer to ml_sensor_raw_data_t into which the four raw sensor
  values generated by lat cells a, b, and c, will be written.

  If the \a dev_hdl argument is not a valid device handle, this function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \param sensorRaw The raw values (voltages) for lat cell a, b, and c.
  \return \c ML_STATUS_OK ( 0 ) upon success, or\n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n

  \sa \c ml_GetCellPosition()
*/

int ml_GetRawCellData(ml_device_handle_t dev_hdl,
		      ml_sensor_raw_data_t * sensorRaw)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_GETRAWCELLDATA,
			      NULL, sensorRaw);
}

/*! \fn int ml_SetAuxDac(ml_device_handle_t dev_hdl, float voltage)
  \brief Outputs voltage to the Aux Diagnostic Output.

  The first argument, \a dev_hdl is the device handle
  of the MLHI controller to which this command is directed.

  The second argument, \a voltage is the voltage outputted. 
  The range of voltage should be [-10,10]. 
  If \a voltage is smaller than -10, -10 is output.
  Similarly, 10 is output if \a voltage exceeds 10.

  If the given \a dev_hdl is invalid, then the function will return
  \c ML_STATUS_INVALID_DEVICE_HANDLE and no values will be read.

  \param dev_hdl The device handle of the MLHI controller to which this command is directed.
  \return \c ML_STATUS_OK ( 0 ) upon success, \n
  \c ML_STATUS_INVALID_DEVICE_HANDLE if the given \a dev_hdl is not a valid device handle referring
  to a currently active connection.\n

*/

int ml_SetAuxDac(ml_device_handle_t dev_hdl, float voltage)
{
  return invoke_remote_function(dev_hdl->socket_ml_func, dev_hdl, 
			      ML_FUNCTION_ID_SETAUXDAC,
			      &voltage, NULL);
}




/**
   @}
*/

