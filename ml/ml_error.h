/*! \file ml_error.h
  \brief This header file defines the error codes used by the MLHI.

  \author Matthew Pucevich                                 
  \date   Created: July 6, 2005  \n  Last Revision: July 14, 2005

  <B>Supervisor:</B>                               \n 
  Dr. Ralph Hollis                                 \n\n
  <B>Location:</B>                                 \n
  Carnegie Mellon University, Robotics Institute:  \n
   Microdynamic Systems Laboratory                 \n
*/


#ifndef _ML_ERROR_H_
#define _ML_ERROR_H_

#ifdef __cplusplus
extern "C"{
#endif

#define ML_STATUS_OK                   0x0000 /*!< SUCCESS - no error */

// DATA ERRORS
#define ML_STATUS_DATA_GAIN_OUT_OF_RANGE    0x0001 /*!< requested gain value is outside maximum limits */
#define ML_STATUS_DATA_GAIN_FORMAT          0x0002 /*!< data is not in the desired format */
#define ML_STATUS_DATA_GAIN_INVALID_TYPE    0x0003 /*!< ?? */
#define ML_STATUS_DATA_AXIS_INVALID         0x0004 /*!< Axis is out of range - must be between 0 and 5 */
#define ML_STATUS_DATA_POS_OUT_OF_RANGE     0x0005 /*!< The requested position value for one or more axes are outside maximum limits */
#define ML_STATUS_DATA_FORCE_OUT_OF_RANGE   0x0006 /*!< The requested force values for one or more axes are outside maximum limits */
#define ML_STATUS_DATA_CURRENT_OUT_OF_RANGE 0x0007 /*!< The requested current values for one or more axes are outside maximum limits */
#define ML_STATUS_DATA_TIME_OUT_OF_RANGE    0x0008 /*!< The requested time value is outside maximum limits */
#define ML_STATUS_INVALID_DEVICE_HANDLE          0x0009 /*!< */
#define ML_STATUS_DATA_SPEED_OUT_OF_RANGE     0x000a /*!<  speed must be non negative */
#define ML_STATUS_NULL_POINTER              0x000f /*!< A pointer parameter has a null value, and does not point to a valid parameter */

// COMMUNICATION ERRORS
#define ML_STATUS_COMM_CONNECTION           0x0010 /*!< A network communication problem has occurred */
#define ML_STATUS_COMM_DUPLICATE_COMMAND    0x0020
#define ML_STATUS_COMM_RECVONLY_MODE        0x0030
#define ML_STATUS_COMM_DUPLICATE_QUEUE      0x0040
#define ML_STATUS_COMM_NOT_QUEUEING         0x0050

// Resource errors

#define ML_STATUS_MEMORY_EXHAUSTED	  0x0060 /*!< internal controller error - unable to allocate memory */
#define ML_STATUS_THREAD_ERROR		  0x0061 /*!< unable to start a thread */
#define ML_STATUS_BLOCK_MATCH_ERROR         0x0062 /*!< A previously registered entity does not match any new request */

// Hardware / Flotor errors
#define ML_STATUS_FLOTOR_LANDED             0x0100
#define ML_STATUS_FLOTOR_TAKEN_OFF          0x0200
#define ML_STATUS_FLOTOR_OFFLINE            0x0300
#define ML_STATUS_FLOTOR_OVERHEAT           0x0400

//MLHI controller internal errors
#define ML_STATUS_MLHI_INT_TIMER            0x0800 /*!< internal error - timer cannot be set or started */
#define ML_STATUS_MLHI_REPLACE		  0x0801   /*!< internal notification - new entity has overwritten a prior, old entity */

#ifdef __cplusplus
}
#endif

#endif
