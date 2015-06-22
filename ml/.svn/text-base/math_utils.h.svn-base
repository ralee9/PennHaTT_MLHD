/*
 *
 * Oct.23 2007
 * Author: Kei Usui
 * utilities to transform between two coordinate frames
 *
 */


#ifndef __MATH_UTILS_H__
#define __MATH_UTILS_H__

#ifdef __cplusplus
extern "C"{
#endif

static const float DEG_TO_RAD = 3.14159265358979/180.0;
static const float RAD_TO_DEG = 180.0 / 3.14159265358979;

void rotate_default_to_user(float* coord_transformed, const float* const coord_org,
			    const float* const rotation);//, bool in_degrees=true);
void rotate_user_to_default(float* coord_transformed, const float* const coord_org,
			    const float* const rotation);//, bool in_degrees=true);

void transform_default_to_user(float* coord_transformed, const float* const coord_org,
			       const float* const translation);//, bool in_degrees=true);
void transform_user_to_default(float* coord_transformed, const float* const coord_org,
			       const float* const translation);//, bool in_degrees=true);

#ifdef __cplusplus
}
#endif

#endif
