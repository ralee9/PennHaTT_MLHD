/*! \file ml_api_common.h

  \brief This header file defines MLHI-specific common data types
  which are used by both RPC/client software and server-side software.

    \author Mark Dzmura
    \date   Created: May 15, 2007

    <B>Location:</B>   \n
    Carnegie Mellon University, Robotics Institute: \n
    Microdynamic Systems Laboratory \n
*/

#ifndef __ML_API_COMMON_H__
#define __ML_API_COMMON_H__

#include "ml_types.h"

/*! generic function pointer */
typedef void (*generic_func_t)(void);

// forward declarations to enable mutual dependence of data structures
struct ml_device_t;
typedef ml_device_t *ml_device_handle_t;
struct ml_button_t;
struct ml_boundary_violation_t;
struct ml_fault_t;
struct ml_button_t;
struct ml_temps_t;
struct ml_position_t;
struct ml_forces_t;

/* Add Callback Step #1: Add a callback function declaration here. */

/*! function prototype for button state change callback */
typedef int (*button_state_callback_func_t)(ml_device_handle_t, ml_button_t);

/*! function prototype for boundary violation callback */
typedef int (*boundary_callback_func_t)(ml_device_handle_t, ml_boundary_violation_t *);

/*! function prototype for coil overtemperature callback */
typedef int (*coil_overtemp_callback_func_t)(ml_device_handle_t, ml_temps_t *);

/*! function prototype for fault state change callback */
typedef int (*fault_callback_func_t)(ml_device_handle_t, ml_fault_t);

/*! function prototype for tick callback */
typedef int (*tick_callback_func_t)(ml_device_handle_t, ml_position_t *);

typedef int (*control_loop_func_t)(ml_device_handle_t, ml_position_t pos, ml_forces_t force);

#endif // __ML_API_COMMON_H__
