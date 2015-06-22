/*******************************************************************************
 
Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Edited by Randy Lee (2015) to implement the Penn Haptic Texture Toolkit on the MLHD

Module Name:

  Constants.c

Description:

  Defines the constants of the simulation

*******************************************************************************/

#define VISITOR_SPHERE_RADIUS   1.6
#define FIXED_SPHERE_RADIUS     50.0
#define OBJECT_MASS             100.0
//#define STIFFNESS               0.5  /* On range [0,1] of device's max stiffness */
#define STIFFNESS_OMNI0_5x      1.15  /* MLHD translation, using N/mm to match OMNI */
#define STIFFNESS_OMNI1_0x      2.30  /* Full scale omni stiffness */
#define STIFFNESS_OMNI2_0x      4.60  /* Double Omni stiffness */
#define STIFFNESS_OMNI1_5x      2.30*1.50 /* 1.5x OMNI stiffness */

#define EPSILON 0.00001 /* zero, for purposes of calculating distances. */

/******************************************************************************/
