#ifndef __ML_API_1_H__
#define __ML_API_1_H__

#include "ml_api.h"

int invoke_local_function(ml_device_handle_t dev_hdl, ml_function_id_t* func_id,
			  char* args_in, char* args_out);

#endif // __ML_API_1_H__
