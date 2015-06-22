/*! \file ml_api_callback.h
  \brief This is a header file for ml_api_callback.c, which implements the callback api for the Magnetic Levitation Haptic Interface API.

  \author Mark Dzmura
  \date   Created: Apr 19, 2007  \n  Last Revision: Apr 25, 2007

  <B>Supervisor:</B>                               \n 
  Dr. Ralph Hollis                                 \n\n
  <B>Location:</B>                                 \n
  Carnegie Mellon University, Robotics Institute:  \n
   Microdynamic Systems Laboratory                 \n
*/

#ifndef _MLHI_API_CALLBACK_H_
#define _ML_API_CALLBACK_H_

// rpc implementation
#include <rpc/rpc.h>
#include <netinet/in.h>

#include "ml_error.h" // MLHI API error codes
#include "ml_constants.h" // MLHI API custom constants
//#include "ml_types.h" // MLHI API custom types
#include "ml_api_common.h" // MLHI common non-RPC types

// ---- CLIENT-side functions ----

extern int _ml_RegisterCallbackServerWithServer(ml_device_handle_t dev_hdl,
						ml_callback_server_info_t & callback_server_info);

extern int _ml_UnregisterCallbackServerWithServer(ml_device_handle_t dev_hdl,
						  ml_callback_server_info_t & callback_server_info);

extern int _ml_RegisterCallbackWithServer(ml_device_handle_t dev_hdl,
					  ml_callback_type_t callback_type,
					  void (*callback_func)(void));

extern int _ml_UnregisterCallbackWithServer(ml_device_handle_t dev_hdl,
					    ml_callback_type_t callback_type);

// ---- SERVER-side functions ----

extern int _ml_RegisterRemoteCallbackServer(in_addr_t ip_addr,
					    char * host_name,
					    ml_callback_server_info_t * csip,
					    CLIENT * client_hdl);

extern int _ml_UnregisterRemoteCallbackServer(in_addr_t ip_addr,
					      char * host_name,
					      ml_callback_server_info_t * csip,
					      CLIENT ** client_hdl_p);

extern int _ml_RegisterRemoteCallback(in_addr_t ip_addr,
				      char * host_name,
				      ml_callback_info_t * cip);

extern int _ml_UnregisterRemoteCallback(in_addr_t ip_addr,
					char * host_name,
					ml_callback_info_t * cip);

#endif	// _ML_API_CALLBACK_H_
