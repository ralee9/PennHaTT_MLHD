#ifndef __ML_SERIALIZATION__
#define __ML_SERIALIZATION__

#include "ml_api.h"

// use int32_t as the standard int for communication
#ifdef WIN32
typedef int ml_int_t;
typedef unsigned int ml_uint_t;
#else
typedef int32_t ml_int_t;
typedef uint32_t ml_uint_t;
#endif

#ifdef __cplusplus
extern "C"{
#endif


static const int sizeof_ml_int_t = sizeof(ml_int_t);
static const int sizeof_ml_uint_t = sizeof(ml_uint_t);


// -------------- BEGIN handle primitive data types ----------------- //
int serialize_float(char* const serialized_data, const float* const raw_data);
int deserialize_float(const char* const serialized_data, float* const deserialized_data);

int serialize_int(char* const serialized_data, const int* const raw_data);
int deserialize_int(const char* const serialized_data, int* const deserialized_data);

int serialize_uint(char* const serialized_data, const unsigned int* const raw_data);
int deserialize_uint(const char* const serialized_data, unsigned int* const deserialized_data);

int serialize_ints(char* const serialized_data, const int* const raw_data, int num_ints);
int deserialize_ints(const char* const serialized_data, int* const deserialized_data, int num_ints);

int serialize_bool(char*const  serialized_data, const int* const raw_data);
int deserialize_bool(const char* const serialized_data, int* const deserialized_data);
// -------------- END handle primitive data types ----------------- //

// -------------- BEGIN handle ml_* data types ----------------- //
int deserialize_ml_gain_pid_t(const char* const serialized_data, ml_gain_pid_t* const deserialized_data);
int serialize_ml_gain_pid_t(char* const serialized_data, const ml_gain_pid_t* const raw_data);

int deserialize_ml_gain_vec_t(const char* const serialized_data, ml_gain_vec_t* const deserialized_data);
int serialize_ml_gain_vec_t(char* const serialized_data, const ml_gain_vec_t* const raw_data);

int deserialize_ml_button_t(const char* const serialized_data, ml_button_t * deserialized_data);
int serialize_ml_button_t(char* serialized_data, const ml_button_t* const raw_data);

int deserialize_ml_position_t(const char* const serialized_data, ml_position_t* const deserialized_data);
int serialize_ml_position_t(char* const serialized_data, const ml_position_t* const raw_data);

int deserialize_ml_float6_t(const char* const serialized_data, ml_float6_t * deserialized_data);
int serialize_ml_float6_t(char* serialized_data, const ml_float6_t* const raw_data);

int deserialize_ml_axis_mode_vec_t(const char* const serialized_data, ml_axis_mode_vec_t * deserialized_data);
int serialize_ml_axis_mode_vec_t(char* serialized_data, const ml_axis_mode_vec_t* const raw_data);

int deserialize_ml_sensor_t(const char* const serialized_data, ml_sensor_t * deserialized_data);
int serialize_ml_sensor_t(char* serialized_data, const ml_sensor_t* const raw_data);

int deserialize_ml_sensor_raw_data_t(const char* const serialized_data, ml_sensor_raw_data_t * deserialized_data);
int serialize_ml_sensor_raw_data_t(char* serialized_data, const ml_sensor_raw_data_t* const raw_data);

int deserialize_ml_sensor_offset_t(const char* const serialized_data, ml_sensor_offset_t* deserialized_data);
int serialize_ml_sensor_offset_t(char* serialized_data, const ml_sensor_offset_t* const raw_data);

int deserialize_ml_boundary_violation_t(const char* const serialized_data, ml_boundary_violation_t* deserialized_data);
int serialize_ml_boundary_violation_t(char* serialized_data, const ml_boundary_violation_t* const raw_data);

// -------------- END handle ml_* data types ----------------- //


// -------------- BEGIN handle ml_* functions ----------------- //
int dealwith_function_argument_in(char* enc_data, ml_function_id_t ID_ML_FUNC, void* args);
int dealwith_function_argument_out(char* enc_data, ml_function_id_t ID_ML_FUNC, void* args);
// -------------- END handle ml_* functions ----------------- //


// -------------- BEGIN helper data types used for communication ----------------- //
struct ml_set_gain_vec_axes_t
{
  ml_gainset_type_t gainset_type;
  ml_gain_vec_t gains;
};
typedef struct ml_set_gain_vec_axes_t ml_set_gain_vec_axes_t;

struct ml_set_gain_vec_axis_t
{
  ml_gainset_type_t gainset_type;
  ml_axis_index_t axis;
  ml_gain_pid_t gain;
};
typedef struct ml_set_gain_vec_axis_t ml_set_gain_vec_axis_t;

struct ml_get_gain_vec_axis_t
{
  ml_gainset_type_t gainset_type;
  ml_axis_index_t axis;
};
typedef struct ml_get_gain_vec_axis_t ml_get_gain_vec_axis_t;

struct ml_set_gain_vec_single_t
{
  ml_gainset_type_t gainset_type;
  ml_axis_index_t axis;
  ml_gain_type_t gaintype;
  ml_gain_t value;
};
typedef struct ml_set_gain_vec_single_t ml_set_gain_vec_single_t;

struct ml_get_gain_vec_single_t
{
  ml_gainset_type_t gainset_type;
  ml_axis_index_t axis;
  ml_gain_type_t gaintype;
};
typedef struct ml_get_gain_vec_single_t ml_get_gain_vec_single_t;

struct ml_set_force_axis_t
{
  ml_axis_index_t axis;
  ml_force_t force;
};
typedef struct ml_set_force_axis_t ml_set_force_axis_t;

struct ml_constrain_axis_t
{
  ml_axis_index_t axis;
  ml_coord_t minpos;
  ml_coord_t maxpos;
};
typedef struct ml_constrain_axis_t ml_constrain_axis_t;

struct ml_set_lat_cell_offset_t
{
  ml_sensor_offset_t cells[3];
};
typedef struct ml_set_lat_cell_offset_t ml_set_lat_cell_offset_t;

struct ml_get_cell_position_t
{
  ml_sensor_t cells[3];
};
typedef struct ml_get_cell_position_t ml_get_cell_position_t;

// -------------- END helper data types used for communication ----------------- //

#ifdef __cplusplus
}
#endif

#endif // __ML_SERIALIZATION__
