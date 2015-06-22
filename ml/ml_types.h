/*! \file ml_types.h
  \brief This header file contains the various non-standard types used by
  the ml_ API layer and ml_ communication layer code.

  \author Joey Quansheng Liang
  \date   Created: July 2, 2006

  \author Mark Dzmura
  \date   Created: Feb 1, 2007

  \author Kei Usui
  \date   Modified: Nov 26, 2007

  <B>Location:</B>   \n
  Carnegie Mellon University, Robotics Institute: \n
  Microdynamic Systems Laboratory \n
*/

#ifndef __ML_TYPES_H__
#define __ML_TYPES_H__

#ifdef WIN32
#include <Windows.h>
#include <winsock.h>
#else
#include <netinet/in.h>
#include <pthread.h>
#endif

#ifdef WIN32
typedef struct in_addr in_addr;
#define true 1
#define false 0
#endif

#ifdef __cplusplus
extern "C"{
#endif

//****************** BEGIN FORWARD DECLARATIONS **********************//
struct ml_device_t;
struct ml_button_t;
struct ml_boundary_violation_t;
struct ml_float6_t;
struct ml_position_t;
struct ml_fault_t;

typedef struct ml_float6_t ml_float6_t;
/*! vector of per-axis (6) temperatures */
typedef struct ml_float6_t ml_temps_t;
/*! typedef for 3 forces (N) and 3 torques (Nm) */
typedef struct ml_float6_t ml_forces_t;
typedef struct ml_device_t *ml_device_handle_t;
typedef struct ml_button_t ml_button_t;
//****************** END FORWARD DECLARATIONS **********************//



//****************** BEGIN CALLBACK FUNCTIONS **********************//
/*! generic function pointer */
typedef void (*generic_func_t)(void);

/* Add Callback Step #1: Add a callback function declaration here. */

/*! function prototype for button state change callback */
typedef int (*button_state_callback_func_t)(ml_device_handle_t dev_hdl, ml_button_t button);

/*! function prototype for boundary violation callback */
typedef int (*boundary_callback_func_t)(ml_device_handle_t, struct ml_boundary_violation_t *);

/*! function prototype for coil overtemperature callback */
typedef int (*coil_overtemp_callback_func_t)(ml_device_handle_t, ml_temps_t *);

/*! function prototype for fault state change callback */
typedef int (*fault_callback_func_t)(ml_device_handle_t, struct ml_fault_t);

/*! function prototype for tick callback */
typedef int (*tick_callback_func_t)(ml_device_handle_t, struct ml_position_t *);

//****************** END CALLBACK FUNCTIONS **********************//



/*! transient, client handle to MLHI device, returned by ml_Connect() */
// maglev api is designed such that
// 1. a client program can communicate with multiple servers simultaneously
// 2. more than one threads from the same client program cannot communicate with the same server simultanouesly
// 3. more than one client programs can communicate with the same server simultaneously
typedef struct ml_device_t
{
  struct in_addr server_addr; /*< address of the server connected as in_addr */
  int socket_ml_func; /*< socket used for calling server functions */
  int socket_ml_callback; /*< socket used for invoking callbacks */
#ifdef WIN32
  HANDLE callback_thread_id;  /*< handle for callback thread */
  HANDLE callback_thread_mutex; /*< used to wait for the callback thread to exit */
  HANDLE comm_mutex; /*< used to avoid multiple threads talking to the same server simultaneously */
#else
  pthread_mutex_t comm_mutex; /*< used to avoid multiple clients talking to the same server simultaneously */
  pthread_t callback_thread_id; /*< handle for callback thread */
#endif
  button_state_callback_func_t button_state_callback_func; /*< pointer to button callback handler function */
  boundary_callback_func_t boundary_callback_func; /*< pointer to boundary callback handler function */
  coil_overtemp_callback_func_t coil_overtemp_callback_func; /*< pointer to coil overtemp callback handler function */
  fault_callback_func_t fault_callback_func; /*< pointer to fault callback handler function */
  tick_callback_func_t tick_callback_func; /*< pointer to tick callback handler function */
};
//typedef struct ml_device_t ml_device_t;


/*! Enumeration capable of representing each of the valid fault types. 
  Note that they increase by a factor of two like 1,2,4,8....
*/
enum ml_fault_type_t
  {
    ML_FAULT_TYPE_CLEAR=0,	/*!< Fault condition CLEARED */

    ML_FAULT_TYPE_SENSOR_OUT_OF_RANGE=1, /*!< Fault: position sensor has no result */
    ML_FAULT_TYPE_COIL_OVERTEMP=2,	/*!< Fault: temperature model for one or more coils is high */
    ML_FAULT_TYPE_COIL_OVERCURRENT=4,	/*!< Fault: total current running on one or more coils over some period is high (approximately 2A on average over 1 minute) */
    ML_FAULT_TYPE_FLOTOR_OVERSPEED=8	/*!< Fault: speed of the flotor is high */    
  };

/*! Enumeration capable of representing each of the valid callback types.
 */

enum ml_callback_type_t
  {
    ML_CALLBACK_TYPE_BUTTON_CHANGED=0, /*!< one or more buttons changed position  */
    ML_CALLBACK_TYPE_BOUNDARY,	     /*!< flotor position has violated the spherical boundary  */
    ML_CALLBACK_TYPE_COIL_OVERTEMP,  /*!< a coil's temperature model indicates over-temperature  */
    ML_CALLBACK_TYPE_FAULT,	     /*!< controller fault state has changed  */
    ML_CALLBACK_TYPE_TICK,	     /*!< controller servo interval (tick) has started */
    ML_CALLBACK_TYPE_SHUTDOWNCLIENT, /*!< internal command used to shut down client callback thread */

    ML_CALLBACK_TYPE_SHUTDOWN	/*!< internal command used to shut down event monitor  */
  };

/*! Enumeration capable of representing each of the valid axis modes.
  When the MLHI device is in normal operation, each of the six
  flotor axis is in one of the legal axis modes: normal, locked, or
  constrained.
*/

enum ml_axis_mode_t
  {
    ML_AXIS_MODE_NORMAL,	/*!< Axis position is under normal PID control  */
    ML_AXIS_MODE_LOCKED,	/*!< Axis position is locked to a specific position */
    ML_AXIS_MODE_CONSTRAINED,	/*!< Axis position is constrained between lower and upper values */
    ML_AXIS_MODE_BOUNDARY	/*!< Internal use only */
  };

/*! Enumeration capable of representing each of the valid gain set types.  A gainset is a complete set of PID controller parameters, including Kp, Ki, and Kd gains and feedforward force term, for each of the six flotor axes.
 */

enum ml_gainset_type_t
  {
    ML_GAINSET_TYPE_NORMAL = 0,	/*!< Designates the set of gains for normal (PID control) axis mode */
    ML_GAINSET_TYPE_LOCKED,	/*!< Designates the set of gains used for a locked axis  */
    ML_GAINSET_TYPE_CONSTRAINED, /*!< Designates the set of gains used outside constraints */
    ML_GAINSET_TYPE_BOUNDARY	 /*!< Designates the set of gains used to determine boundary forces  */
  };

typedef enum ml_gainset_type_t ml_gainset_type_t;

/*! Enumeration defining symbolic constants to name individual axes.  Any API call specifying a specific axis takes a simple integer parameter to designate the desired axis.  This enumeration allows axes to be designated with symbolic names.
 */

enum ml_axis_index_t
  {

    ML_AXIS_0 = 0,		/*!< "X position" Axis - Axis #0 */
    ML_AXIS_X = ML_AXIS_0,	/*!< (Alias for "X position" Axis - Axis #0) */
    ML_AXIS_X_POS = ML_AXIS_0,	/*!< (Alias for "X position" Axis - Axis #0)\n */

    ML_AXIS_1 = 1,		/*!< "Y position" Axis - Axis #1 */
    ML_AXIS_Y = ML_AXIS_1,	/*!< (Alias for "Y position" Axis - Axis #1) */
    ML_AXIS_Y_POS = ML_AXIS_1,	/*!< (Alias for "Y position" Axis - Axis #1)\n */

    ML_AXIS_2 = 2,		/*!< "Z position" Axis - Axis #2 */
    ML_AXIS_Z = ML_AXIS_2,	/*!< (Alias for "Z position" Axis - Axis #2) */
    ML_AXIS_Z_POS = ML_AXIS_2,	/*!< (Alias for "Z position" Axis - Axis #2)\n */

    ML_AXIS_3 = 3,		/*!< "X rotation" Axis - Axis #3 */
    ML_AXIS_X_ROT = ML_AXIS_3,	/*!< (Alias for "X rotation" Axis - Axis #3) */
    ML_AXIS_PITCH = ML_AXIS_3,	/*!< (Alias for "X rotation" Axis - Axis #3)\n */

    ML_AXIS_4 = 4,		/*!< "Y rotation" Axis - Axis #4 */
    ML_AXIS_Y_ROT = ML_AXIS_4,	/*!< (Alias for "Y rotation" Axis - Axis #4) */
    ML_AXIS_ROLL = ML_AXIS_4,	/*!< (Alias for "Y rotation" Axis - Axis #4)\n */

    ML_AXIS_5 = 5,		/*!< "Z rotation" Axis - Axis #5 */
    ML_AXIS_Z_ROT = ML_AXIS_5,	/*!< (Alias for "Z rotation" Axis - Axis #5) */
    ML_AXIS_YAW = ML_AXIS_5,	/*!< (Alias for "Z rotation" Axis - Axis #5)\n */

    ML_AXIS_GRASP		/*!< "Grasp" Axis - Axis #6 */
  };
typedef enum ml_axis_index_t ml_axis_index_t;

/*! Enumeration defining per-axis PID controller parameters (gains, etc.) */
enum ml_gain_type_t
  {
    ML_GAIN_TYPE_P=0,		/*!< Designates proportional gain type: Kp */
    ML_GAIN_TYPE_I,		/*!< Designates integral gain type: Ki */
    ML_GAIN_TYPE_D,		/*!< Designates derivative gain type: Kd */
    ML_GAIN_TYPE_FF		/*!< Designates feedforward force type: FF */
  };
typedef enum ml_gain_type_t ml_gain_type_t;


/*! Enumeration capable of representing each of the functions. 
 */
enum ml_function_id_t
  {
    ML_FUNCTION_ID_CONNECT=0,
    ML_FUNCTION_ID_CONSTRAINAXIS,
    ML_FUNCTION_ID_DEFYGRAVITY,
    ML_FUNCTION_ID_DISCONNECT,
    ML_FUNCTION_ID_FINDGRAVITY,
    ML_FUNCTION_ID_GETACTUALPOSITION,
    ML_FUNCTION_ID_GETAXISMODES,
    ML_FUNCTION_ID_GETBOUNDARYRADIUS,
    ML_FUNCTION_ID_GETBUTTONSTATUS,
    ML_FUNCTION_ID_GETCELLPOSITION,
    ML_FUNCTION_ID_GETCURRENTLIMITS,
    ML_FUNCTION_ID_GETCURRENTS,
    ML_FUNCTION_ID_GETDESIREDPOSITION,
    ML_FUNCTION_ID_GETESTOP,
    ML_FUNCTION_ID_GETFAULT,
    ML_FUNCTION_ID_GETFORCEAXIS,
    ML_FUNCTION_ID_GETFORCES,
    ML_FUNCTION_ID_GETFRAME,
    ML_FUNCTION_ID_GETGAINVECAXES,
    ML_FUNCTION_ID_GETGAINVECAXIS,
    ML_FUNCTION_ID_GETGAINVECGRASP,
    ML_FUNCTION_ID_GETGAINVECSINGLE,
    ML_FUNCTION_ID_GETGRASPCURRENT,
    ML_FUNCTION_ID_GETGRASPCURRENTLIMIT,
    ML_FUNCTION_ID_GETGRAVITY,
    ML_FUNCTION_ID_GETLATCELLLIMIT,
    ML_FUNCTION_ID_GETLATCELLOFFSET,
    ML_FUNCTION_ID_GETRAWCELLDATA,
    ML_FUNCTION_ID_GETSERVOFREQUENCY,
    ML_FUNCTION_ID_GETSPEEDLIMITS,
    ML_FUNCTION_ID_GETTEMP,
    ML_FUNCTION_ID_GETVELOCITY,
    ML_FUNCTION_ID_GETVERSION,
    ML_FUNCTION_ID_LAND,
    ML_FUNCTION_ID_LOCKAXIS,
    ML_FUNCTION_ID_RECORDVOLTAGES,
    ML_FUNCTION_ID_REGISTERCALLBACKBUTTONPRESSED,
    ML_FUNCTION_ID_REGISTERCALLBACKFAULT,
    ML_FUNCTION_ID_REGISTERCALLBACKFLOTORBOUNDARYVIOLATION,
    ML_FUNCTION_ID_REGISTERCALLBACKOVERTEMP,
    ML_FUNCTION_ID_REGISTERCALLBACKTICK,
    ML_FUNCTION_ID_RESETFAULT,
    ML_FUNCTION_ID_RESETGAINVECAXES,
    ML_FUNCTION_ID_SETAUXDAC,
    ML_FUNCTION_ID_SETBOUNDARYRADIUS,
    ML_FUNCTION_ID_SETCURRENTLIMITS,
    ML_FUNCTION_ID_SETCURRENTS,
    ML_FUNCTION_ID_SETDESIREDPOSITION,
    ML_FUNCTION_ID_SETFORCEAXIS,
    ML_FUNCTION_ID_SETFORCES,
    ML_FUNCTION_ID_SETFRAME,
    ML_FUNCTION_ID_SETGAINVECAXES,
    ML_FUNCTION_ID_SETGAINVECAXIS,
    ML_FUNCTION_ID_SETGAINVECGRASP,
    ML_FUNCTION_ID_SETGAINVECSINGLE,
    ML_FUNCTION_ID_SETGRASPCURRENT,
    ML_FUNCTION_ID_SETGRASPCURRENTLIMIT,
    ML_FUNCTION_ID_SETGRAVITY,
    ML_FUNCTION_ID_SETLATCELLOFFSET,
    ML_FUNCTION_ID_SETSERVOFREQUENCY,
    ML_FUNCTION_ID_SETSPEEDLIMITS,
    ML_FUNCTION_ID_SETVELOCITY,
    ML_FUNCTION_ID_SHUTDOWN,
    ML_FUNCTION_ID_TAKEOFF,
    ML_FUNCTION_ID_UNLOCKAXIS,
    ML_FUNCTION_ID_UNREGISTERCALLBACKBUTTONPRESSED,
    ML_FUNCTION_ID_UNREGISTERCALLBACKFAULT,
    ML_FUNCTION_ID_UNREGISTERCALLBACKFLOTORBOUNDARYVIOLATION,
    ML_FUNCTION_ID_UNREGISTERCALLBACKOVERTEMP,
    ML_FUNCTION_ID_UNREGISTERCALLBACKTICK,
    ML_FUNCTION_ID_UPDATEPOSITION,
    NON_ML_FUNCTION_ID_TERMINATECALLBACKTHREADCLIENT,
    NON_ML_FUNCTION_ID_TERMINATECALLBACKTHREADSERVER,
    NON_ML_FUNCTION_ID_MAPCALLBACKSOCKET,
    NON_ML_FUNCTION_ID_CALLBACK_TYPE_BUTTON_CHANGED_HANDLER,
    NON_ML_FUNCTION_ID_CALLBACK_TYPE_BOUNDARY_HANDLER,
    NON_ML_FUNCTION_ID_CALLBACK_TYPE_COIL_OVERTEMP_HANDLER,
    NON_ML_FUNCTION_ID_CALLBACK_TYPE_FAULT_HANDLER,
    NON_ML_FUNCTION_ID_CALLBACK_TYPE_TICK_HANDLER,
    NON_ML_FUNCTION_ID_CALLBACK_TYPE_SHUTDOWN	/*!< internal command used to shut down event monitor  */
  };
typedef enum ml_function_id_t ml_function_id_t;

/* ---------- gain types ---------- */

/*! simple gain type */
typedef float ml_gain_t;

/*! set of gains (and other params) for simple PID */
struct ml_gain_pid_t
{
  ml_gain_t p;		/*!< proportional gain value: Kp */
  ml_gain_t i;		/*!< integral gain value: Ki */
  ml_gain_t d;		/*!< derivative gain value: Kd */
  ml_gain_t ff;		/*!< feedforward force value: FF */
};
typedef struct ml_gain_pid_t ml_gain_pid_t;

/*! Vector of per-axis (6) PID controller parameter structures */
struct ml_gain_vec_t
{
  struct ml_gain_pid_t values[6];	/*!< per-axis gains  */
};
typedef struct ml_gain_vec_t ml_gain_vec_t;

/* --------------------------------
 * interactive type
 * --------------------------------
 */

/*! struct for the status of the two buttons on the flotor's handle */
struct ml_button_t
{
  int left;		/*!< status of left button: if non-zero, button is depressed */
  int right;		/*!< status of right button: if non-zero, button is depressed */
};
//typedef struct ml_button_t ml_button_t;

/*! struct for the fault status of the device */
struct ml_fault_t
{
  unsigned int value;		/*!< bitmap indicating the fault status of the device */
};
/*! typedef for the status of the two buttons on the flotor's handle */
typedef struct ml_fault_t ml_fault_t;


/*! typedef for the coordination */
typedef float ml_coord_t;

/*! typedef for the temperature */
typedef float ml_temp_t;

/*! typedef for the force (N) or torque (Nm) */
typedef float ml_force_t;

/*! struct for flotor's world coordinate position */
struct ml_position_t
{
  int   isOutOfRange;		/*!< if true, cartesian position is outside legal flotor envelope */
  float values[6];     	/*!< array of per-axis values */
};

/*! typedef for flotor's world coordinate position */
typedef struct ml_position_t ml_position_t;

struct ml_boundary_violation_t
{
  int values[7];
};
typedef struct ml_boundary_violation_t ml_boundary_violation_t;


/*! generic struct with 6 float array in it */
typedef struct ml_float6_t
{
  float values[6];		/*!< array of per-axis values  */
} ml_float6_6;

/*! typedef for velocities on x, y, z, and on the angle of x, y, z */
typedef struct ml_float6_t ml_velocities_t;

/*! struct containing 6 ml_axis_mode_t's */
struct ml_axis_mode_vec_t
{
  enum ml_axis_mode_t axis_mode[6]; /*!< array of per-axis axis modes */
};
typedef struct ml_axis_mode_vec_t ml_axis_mode_vec_t;


/*! typedef for ml_axis_mode_vec_t
  typedef struct ml_axis_mode_vec_t ml_axis_mode_vec_t;

  /* -------------- hardware type ---------------- */

/*! struct for sensor position */
struct ml_sensor_t
{
  float x;			/*!< sensor calculated x coordinate */
  float y;			/*!< sensor calculated y coordinate */
  int isValid;		/*!< if true, sensor x/y coord can be trusted */
};

/*! typedef for sensor position */
typedef struct ml_sensor_t ml_sensor_t;

/*! struct for sensor position */
struct ml_sensor_offset_t
{
  float x;                    /*!< sensor offset, x coordinate */
  float y;                    /*!< sensor offset, y coordinate */
  float t;                    /*!< sensor offset, rotation in xy plane */
};

/*! typedef for sensor offset */
typedef struct ml_sensor_offset_t ml_sensor_offset_t;


/*! struct for 3 sensors' raw data, to store 12 voltages */
struct ml_sensor_raw_data_t
{
  float values[12];		/*!< 12 raw values from 3 optical position sensing devices (PSD's) */
};

/*! typedef for sensors' raw data */
typedef struct ml_sensor_raw_data_t ml_sensor_raw_data_t;


/*! typedef for the current, in ampere */
typedef float  ml_current_t;

/*! typedef for the currents of 6 coils, in ampere */
typedef struct ml_float6_t ml_currents_t;

#ifdef __cplusplus
}
#endif

#endif /* __ML_TYPES__H__ */
