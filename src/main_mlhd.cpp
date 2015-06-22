/*****************************************************************************
	Penn Haptic Texture Toolkit Demo

	Adapted for the MLHD
	
	Randy Lee (ral63@pitt.com)
	Visualization and Image Analysis Laboratory
	Department of Bioengineering
	University of Pittsburgh

	Ralph Hollis
	Butterfly Haptics (Pittsburgh, PA)
	Microdynamic Systems Laboratory
	Carnegie Mellon University

-------------------

	Original Source:
	Penn Haptic Texture Toolkit 
		by
	Heather Culbertson (hculb@seas.upenn.edu, +1 215-573-6748) 
	Juan Jose Lopez Delgado,
	Katherine Kuchenbecker, (kuchenbe@seas.upenn.edu, +1 215-573-2786)
	(http://repository.upenn.edu/meam_papers/299/)

	COPYRIGHT AND PERMISSION NOTICE
	Penn Software The Penn Haptic Texture Toolkit
	Copyright (C) 2013 The Trustees of the University of Pennsylvania
	All rights reserved.

	See copyright and permission notice at the end of this file.

	This code is based on the original TexturePad haptic rendering system 
	designed by Joseph M. Romano.

-------------------

	Function:

	Use recorded force and vibration information to recreate textures of 
	different materials. Original recorded data down-sampled from 10 kHz
	to 1kHz using associated MATLAB code.

	Haptic device force and velocity data fed into Delaunay triangulation to 
	estimate normal, tangential, and vibratory texture forces in real-time.

*****************************************************************************/

/** Penn Haptic Toolkit **/
// Standard C++ Libraries
#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <time.h>
/* Boost added as external dependecy in Windows VS soln (available native in Linux)
   Win32: Installed in C:/Program Files/boost_1_57_0  */
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

// Custom Headers
#include "shared.h"                 // Shared global variables
#include "sharedInit.h"             // Shared global variables
#include "AccSynthHashMatrix.h"     // Header for AccSynthHashMatrix.cpp src file
#include "autoGenHashMatrix.h"      // Reads and parses XML model file, generates hash matrix

// File control - debugging
#include <fstream>
#include <cstdlib>


/** MLHD **/
// Standard C++ Libraries
#include <stdio.h>
//#include <conio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

// MLHD API
#include "../ml/ml_api.h"


/** GUI/Visualization Libraries **/
#if defined(WIN32) || defined(linux)
//# include "../GL/glut.h" //original win32 MLHD implementation
# include <GL/glut.h>
#elif defined(__APPLE__)
# include <GLUT/glut.h>
#endif

#if defined(linux)
# include <sys/time.h>
#endif

/** OMNI Helper Headers **/
#include "Constants.h"        // Constants for mat'l stiffness and flotor graphics

/******************************************************************************
Globals, declarations
******************************************************************************/
/** Penn Haptic Texture Toolkit Declarations **/
/* Current button state (OMNI)*/
int btn[3] = { 0, 0, 0 };

/* Current mouse position */
int mouse_x, mouse_y;

/* Dynamics simulation */
// Graphics & visualizations
void displayFunction(void);
void handleMenu(int);
void handleMouse(int, int, int, int);
void initGlut(int argc, char* argv[]);
void initGraphics(const float LLB[3], const float TRF[3]);

// sphere graphics & flat texture surface
void displayVisitorSphere(GLUquadricObj* quadObj, float position[3]);
void displayTextureSurface();

void doGraphicsState();
void loadTexture();
GLuint loadBMP_custom(const char * imagepath);
void printtext(int x, int y, std::string String, bool sizeL);
void processNormalKeys(unsigned char key, int x, int y);

/* Haptic device position, velocity, prior velocity, used in callback */
// ml_float6 for position to match velocity containers
ml_float6_t priorPosition;
ml_velocities_t priorVelocity;
ml_velocities_t priorpriorVelocity;

AccSynthHashMatrix mymatrix;
boost::mt19937 rng;

// time (linux implementation)
struct timeval tv0;
struct timeval tv1;
time_t starttime;
double oldTime = 0;
long long counter = 0; // debug counter

//pAudio variables
std::vector <float> outputHist;
std::vector <float> excitationHist;

// debug variables
bool debug_flag = false;
int loopCount = 0;
int debugLoopCount = 0;


/** MLHD Declarations **/
static ml_device_handle_t Device_Handle = NULL;
float Servo_Frequency = 1000.0f;    // servo frequency in Hz
float Boundary_Radius = 0.012f;	    // radius of the bounding sphere in meters 
char* Default_IP = "192.168.0.2";   // default ip address to be connected

int  InitializeMLHD(char* Server_Address, ml_device_handle_t *device_hdl);
void DisconnectMLHD(ml_device_handle_t device_hdl);
void SmoothTransitionTo(ml_device_handle_t dev_hdl, ml_position_t desiredPosition);

int tick_callback_handler(ml_device_handle_t dev_hdl, ml_position_t* pos);

// GLUT callback functions
void Resize(int width, int height);

// Global MLHD variables
// Gravity value obtained from ml_DefyGravity()
ml_forces_t Gravity;

// Maximum MLHD active workspace
// OMNI workspace: +x = right of user, +y = above origin, +z = closest to user
static float ConstrainXAxisMin = -0.008;    // left of origin
static float ConstrainXAxisMax =  0.008;    // right of origin
static float ConstrainYAxisMin = -0.008;    // closest to user (out)
static float ConstrainYAxisMax =  0.008;    // furthest from user (in)
static float ConstrainZAxisMin = -0.008;    // below origin (vertical)
static float ConstrainZAxisMax =  0.008;    // above origin (vertical)

static ml_position_t   curr_Position;
static ml_velocities_t curr_Velocity;
static ml_forces_t     curr_Force;
static ml_gain_pid_t   VecGains;
time_t t1, t2, t3;

static float curr_p_x, curr_p_y, curr_p_z;
static float curr_v_x, curr_v_y, curr_v_z;
static float gravityX, gravityY, gravityZ;
static int   ilower, iupper;

double STIFFNESS[4] = {STIFFNESS_OMNI2_0x, STIFFNESS_OMNI1_5x,
		       STIFFNESS_OMNI1_0x, STIFFNESS_OMNI0_5x};
double stiffness = STIFFNESS[0];	// units = N/mm

float ForceX, ForceY, ForceZ;
float Radius;

// Globals for save/load gravity functions
float xfrac, yfrac, zfrac;
float indexFloor;            // float to match modf()
int  xindex, yindex, zindex; // if use modf() for gravity calibration

char str[1024];



/******************************************************************************
MLHD Function:	Initialize the Maglev device
******************************************************************************/
int InitializeMLHD(char* Server_Address, ml_device_handle_t *device_hdl)
{
	
    ml_fault_t fault;
    int flag;   // used to Defy Gravity

    // to create a connection between the program and the MLHI controller
    if(ML_STATUS_OK != ml_Connect(device_hdl, Server_Address)) {
        fprintf (stderr, "\nError!!!! Unable to connect to the server (%s).\n",
                 Server_Address);
        return -1;
    }

    printf("Successfully connected to MLHD server!\n");
    printf("Taking off.\n");

    // to take off the flotor
    do{
        do{
            ml_ResetFault(*device_hdl);
            //Sleep(500); //Win32 implementation, takes ms
            usleep(500 * 1000); // UNIX implementation, takes us
            ml_GetFault(*device_hdl, &fault);
         }while(ML_STATUS_OK != fault.value);

        // To take off the flotor and reset gains to 0
        flag = ml_Takeoff(*device_hdl);
    }while(ML_STATUS_OK != flag);

    // set the device parameters
    ml_SetServoFrequency(*device_hdl, Servo_Frequency);
    ml_SetBoundaryRadius(*device_hdl, Boundary_Radius);
    printf("Successfully set servo frequency and boundary radius.\n");

    // to defy gravity
    printf("Finding gravity.\n");
    flag = 0;
    if (ML_STATUS_OK != ml_FindGravity(*device_hdl, &Gravity)) flag++;
    else{
        printf("Gravity vector found: (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f)\n",
               Gravity.values[0], Gravity.values[1], Gravity.values[2],
               Gravity.values[3], Gravity.values[4], Gravity.values[5]);

        printf("Decreasing gravity in z by one to give flotor mass.\n");
        Gravity.values[2] -= 1;
        printf(" gravityZ = %.2f", Gravity.values[2]);

        if (ML_STATUS_OK != ml_SetGravity(*device_hdl, Gravity)) flag++;
        else{
            printf("Gravity vector successfully set into feedforward gains.\n");
            if (ML_STATUS_OK != ml_DefyGravity(*device_hdl)) flag++;
        }
    }
    if (flag != 0){
        fprintf(stderr, "**Error!!!! Failed to defy gravity.**\nDisconnecting.\n");
        // reset the MLHD to the default position/orientation with the default
        // set of gains.
        ml_Land (*device_hdl);
        // disconnect from the MLHD server
        ml_Disconnect(*device_hdl);
        return -2;
    }

    printf("Successfully set gravity!\n");
    printf("\nInitialization complete!\n");

    return 0;
}

/******************************************************************************
MLHD Function:	Disconnect the client application from the Maglev device
******************************************************************************/
void DisconnectMLHD(ml_device_handle_t device_hdl)
{
    // unregister the previously registered callback function
    ml_UnregisterCallbackTick(device_hdl);
    //Sleep(25);        // Win32 implementation, takes ms
    usleep(25*1000);    // unix implementation, takes us

    // unlock all axes
    ml_UnlockAxis (device_hdl, ML_AXIS_X_ROT);
    ml_UnlockAxis (device_hdl, ML_AXIS_Y_ROT);
    ml_UnlockAxis (device_hdl, ML_AXIS_Z_ROT);
    ml_UnlockAxis (device_hdl, ML_AXIS_X_POS);
    ml_UnlockAxis (device_hdl, ML_AXIS_Y_POS);
    ml_UnlockAxis (device_hdl, ML_AXIS_Z_POS);

    // reset the device to the default status
    ml_Land (device_hdl);
    // disconnect from server
    ml_Disconnect(device_hdl);

    return ;
}

/******************************************************************************
MLHD Function:	Smoothly move the flotor to a specified location 
******************************************************************************/
void  SmoothTransitionTo(ml_device_handle_t dev_hdl, ml_position_t desiredPosition)
{
  int i,j,Steps = 125;
  ml_position_t   curPosition, tgtPosition;

  ml_GetActualPosition(dev_hdl, &curPosition);
  for(i=0;i<=Steps;i++){
      for(j=0;j<6;j++){
          tgtPosition.values[j]= curPosition.values[j] + (desiredPosition.values[j]-curPosition.values[j])*(float)i/(float)Steps;
      }

      ml_SetDesiredPosition(dev_hdl, tgtPosition);
      // wait for 2ms until the action is done
      //Sleep(2);     // win32
      usleep(2*1000); // uNIX implementation, requires unistd.h
  }
  return;
}

/*******************************************************************************
MLHD Function:  Resize GLUT windows
*******************************************************************************/
void Resize(int width, int height)
{
    glViewport(0,0,width,height);
    return;
}

/******************************************************************************
Penn Toolkit:   The handler gets called when the process is exiting.
******************************************************************************/
void exitHandler()
{
    printf("Closing application. Resetting position.");

    ml_position_t resetPos;
    for (int i=0;i<6;i++) resetPos.values[i] = 0.0f;
    SmoothTransitionTo(Device_Handle, resetPos);
    usleep(50*1000);
    DisconnectMLHD(Device_Handle);
}

/******************************************************************************
Penn Toolkit:   Popup menu handler
******************************************************************************/
void handleMenu(int ID)
{
    switch(ID) {
        case 0:
          exit(0);
          break;
    }
}

/******************************************************************************
Penn Toolkit:   Mouse events handler
******************************************************************************/
void handleMouse(int b, int s, int x, int y)
{
    if (s == GLUT_DOWN) {
        btn[b] = 1;
        } else {
        btn[b] = 0;
    }

    mouse_x = x;
    mouse_y = glutGet(GLUT_WINDOW_HEIGHT) - y;
}

/******************************************************************************
Penn Toolkit:   Calculate vibrations from texture files
******************************************************************************/
static double vibrations()
{
    double output = 0.0;
    double excitation = 0.0;
    double rgen_mean=0.;
    boost::mt19937 generator;
    
    //Double buffered, if buffer 1:
    if(SynthesisFlag_Buffer1) {
        //generate Gaussian random number with power equal to interpolation model variance
        boost::normal_distribution<> nd(rgen_mean, sqrt(filtVariance_buf1));
        boost::variate_generator<boost::mt19937&,
        boost::normal_distribution<> > var_nor(rng, nd);
        excitation = var_nor();
        output = 0.0;
        
        //if the size of output history is less than the number of AR coefficients, append zeros
        if(outputHist.size()<(unsigned int) MAX_COEFF) {
            int subt = MAX_COEFF - outputHist.size();
            for(int j = 0; j < subt ; j++) {
                outputHist.push_back(0.0);
            }
        }
        //if the size of excitation history is less than the number of MA coefficients, append zeros
        if(excitationHist.size()<(unsigned int) MAX_MACOEFF) {
            int subt = MAX_MACOEFF - excitationHist.size();
            for(int j = 0; j < subt ; j++) {
                excitationHist.push_back(0.0);
            }
        }
        
        //apply AR coefficients to history of output values
        for(int i = 0; i < coeffNum; i++) {
            output += outputHist.at(i) * (-filtCoeff_buf1[i]);
        }
        //if applicable, also apply MA coefficients to history of excitation values
        if(isARMA){
            output += excitation*filtGain_buf1;
            for(int i = 0; i < MAcoeffNum; i++) {
                output += excitationHist.at(i) * (filtMACoeff_buf1[i])*filtGain_buf1;
            }
            
            } else{
            output += excitation;
        }

        //if the size of output history is greater than the number of AR coefficients, make the extra values zero so we're not storing junk
        if(outputHist.size()>(unsigned int) coeffNum) {
            for(unsigned int kk = coeffNum; kk < outputHist.size(); kk++)
            outputHist.at(kk) = 0.0;
        }
        //if the size of excitation history is greater than the number of MA coefficients, make the extra values zero so we're not storing junk
        if(excitationHist.size()>(unsigned int) MAcoeffNum) {
            for(unsigned int kk = MAcoeffNum; kk < excitationHist.size(); kk++)
            excitationHist.at(kk) = 0.0;
        }
        
        } else {//if buffer 2
        //generate Gaussian random number with power equal to interpolation model variance
        boost::normal_distribution<> nd(rgen_mean, sqrt(filtVariance_buf2));
        boost::variate_generator<boost::mt19937&,
        boost::normal_distribution<> > var_nor(rng, nd);
        excitation = var_nor();
        output = 0.0;
        
        //if the size of output history is less than the number of AR coefficients, append zeros
        if(outputHist.size()<(unsigned int) MAX_COEFF) {
            int subt = MAX_COEFF - outputHist.size();
            for(int j = 0; j < subt ; j++) {
                outputHist.push_back(0.0);
            }
        }
        //if the size of excitation history is less than the number of MA coefficients, append zeros
        if(excitationHist.size()<(unsigned int) MAX_MACOEFF) {
            int subt = MAX_MACOEFF - excitationHist.size();
            for(int j = 0; j < subt ; j++) {
                excitationHist.push_back(0.0);
            }
        }
        
        //apply AR coefficients to history of output values
        for(int i = 0; i < coeffNum; i++) {
            output += outputHist.at(i) * (-filtCoeff_buf2[i]);
        }
        //if applicable, also apply MA coefficients to history of excitation values
        if(isARMA){
            output += excitation*filtGain_buf2;
            for(int i = 0; i < MAcoeffNum; i++) {
                output += excitationHist.at(i) * (filtMACoeff_buf2[i])*filtGain_buf2;
            }
            
            } else{
            output += excitation;
        }

        //if the size of output history is greater than the number of AR coefficients, make the extra values zero so we're not storing junk
        if(outputHist.size()>(unsigned int) coeffNum) {
            for(unsigned int kk = coeffNum; kk < outputHist.size(); kk++) {
                outputHist.at(kk) = 0.0;
            }
        }
        //if the size of excitation history is greater than the number of MA coefficients, make the extra values zero so we're not storing junk
        if(excitationHist.size()>(unsigned int) MAcoeffNum) {
            for(unsigned int kk = MAcoeffNum; kk < excitationHist.size(); kk++)
            excitationHist.at(kk) = 0.0;
        }
    }
    
    // remove the last element of our output vector
    outputHist.pop_back();
    excitationHist.pop_back();
    // push our new ouput value onto the front of our vector stack
    outputHist.insert(outputHist.begin(),output);
    excitationHist.insert(excitationHist.begin(),excitation);
    
    return output; //this is the output vibration value (in m/s^2)
}

/******************************************************************************
MLHD Function:  Haptic callback loop
******************************************************************************/
int tick_callback_handler(ml_device_handle_t dev_hdl, ml_position_t* pos)
{

    /** Set the force feedback **/
    double currTime;
    double elapsed;
    double output;
    
    float slopeFric = 0.004; // 0.004
    float vThresh;           // threshold velocity for friction calculation
    vThresh = mu_k/slopeFric;
    
    int direction[3] = {0, 1, 0};   // direction of the vibration - now y-axis (on omni)
    double timer = 0;               // timer, as defined by OH
    double instRate;                // instantaneous rate of update of device

    // MLHD vectors are size 6
    ml_forces_t forceTex;         // the calculated texture force
    ml_forces_t forceFric;        // the friction force
    double forceNorm;             // magnitude of the normal force

    ml_float6_t currPosition;       // current position vector, function scope
    ml_velocities_t currVelocity;   // current velocity vector, function scope
    double distFromCenter;
    ml_velocities_t normalVelocity; // normal vector of the velocity
    ml_float6_t normalVec;          // vector normal to the surface

    ml_velocities_t tanVelocity;    // tangential velocity vector, function scope
    ml_float6_t tanVec;         // vector tangent to direction of travel
    ml_float6_t textureVec;     // direction to display texture force (binormal to normalVec and tanVec)


    /** Calculate current flotor velocity using filtered position **/
    // read the current status of the flotor
    ml_GetActualPosition(dev_hdl, &curr_Position);
    //ml_GetVelocity (dev_hdl, &curr_Velocity); // calculate velocity from filtered position instead
    ml_GetForces(dev_hdl, &curr_Force);

    curr_p_x = curr_Position.values[0];
    curr_p_y = curr_Position.values[1];
    curr_p_z = curr_Position.values[2];

    // copy global curr_Position values into function-scope currPosition for
    // filtered velocity calculation
    for (int i=0; i<6; i++){
        currPosition.values[i] = curr_Position.values[i];
    }

    gettimeofday(&tv1, NULL);
    elapsed = (tv1.tv_sec - tv0.tv_sec) + (tv1.tv_usec-tv0.tv_usec)/1000000.0;
    currTime = elapsed;
    
    // second order lowpass filter parameters
    float lambda = 125.0; //cutoff frequency of lowpass filter in rad/s
    float T = currTime - oldTime; //current sampling time
    float w0 = lambda*lambda*T*T/((1.0+T*lambda)*(1.0+T*lambda)); //weight for z^0 term
    float w1 = 2.0/(1.0+lambda*T); //weight for z^-1 term
    float w2 = -1.0/((1.0+T*lambda)*(1.0+T*lambda)); //weight for z^-2 term

    // function scope velocity used to obtain DT coord
    //currVelocity = w0*((currPosition - priorPosition) / T) + w1*priorVelocity + w2*priorpriorVelocity; //second order low-pass filter at 20 Hz
    for (int i=0;i<6;i++){
        currVelocity.values[i] = w0*((currPosition.values[i] - priorPosition.values[i]) / (currTime-oldTime)) + w1*priorVelocity.values[i] + w2*priorpriorVelocity.values[i]; //second order low-pass filter at 20Hz
    }
    
    //priorpriorVelocity = priorVelocity; //store new values for filter in next loop
    // Store new values for filter in next loop
    for (int i=0;i<6;i++){
        priorpriorVelocity.values[i] = priorVelocity.values[i];
    }

    //priorVelocity = currVelocity;
    for (int i=0;i<6;i++){
        priorVelocity.values[i] = currVelocity.values[i];
    }


    /** Calculate the normal vector **/
    // make normalVec point in z direction
    for (int i=0;i<6;i++){
        normalVec.values[i] = 0.0f;
    }
    normalVec.values[2] = 1.0f;        // unit vector in z direction

    distFromCenter = sqrt(pow(curr_p_x, float(2.0)) +
                          pow(curr_p_y, float(2.0)) +
                          pow(curr_p_z, float(2.0)));

    // get current force being applied to user
    ml_forces_t forceVec;
    ml_GetForces(dev_hdl, &forceVec);

    // subtract gravity from each direction to obtain force applied to user
    forceVec.values[0] -= gravityX;
    forceVec.values[1] -= gravityY;
    forceVec.values[2] -= gravityZ;


    /** Compute texture, friction, and normal forces **/
    // texture model -- Flat plane on z = 0 in MLHD
    if (curr_p_z <= 0.0) //(distFromCenter <= (FIXED_SPHERE_RADIUS + VISITOR_SPHERE_RADIUS)) //if touching sphere
    {
        /** Add normal spring force from "penetrating" surface**/
        // Moved to end of callback to reduce interference with vibration and
        // friction calculations
        // - As would have been set by Omni's UpdateEffectorPosition()
        // - Times 1000 because original OMNI implimentation uses mm for position
        // - STIFFNESS = 0.5 from Constants.h, midpoint of hardnest surfaces OMNI can render
        //   --> max stiffness = 2.31N/m in vertical, for STIFFNESS == 1.0
        //   --> STIFFNESS = 1.15N/mm to correspond to STIFFNESS == 0.5
        //forceNorm = STIFFNESS * (0.0 - 1000.0*curr_p_z);


        /** Calculate vibrations and friction forces **/
        /* Vibration calculations use force applied to user as determined by
         * ml_GetForces and filtered velocity
         *
         */

        // Velocity in normal direction, units = m/s
        float normVelocity_dp = normalVec.values[0] * currVelocity.values[0] +
                                normalVec.values[1] * currVelocity.values[1] +
                                normalVec.values[2] * currVelocity.values[2];

        for (int i=0;i<6;i++){
            normalVelocity.values[i] = normVelocity_dp * normalVec.values[i];
        }

        // normal force being applied, units = N
        forceNorm = normalVec.values[0] * forceVec.values[0] +
                    normalVec.values[1] * forceVec.values[1] +
                    normalVec.values[2] * forceVec.values[2];

        // velocity tangent to texture surface, units = m/s
        for (int i=0;i<6;i++){
            tanVelocity.values[i] = currVelocity.values[i] - normalVelocity.values[i];
        }

        float tanVelocity_mag = sqrt(pow(tanVelocity.values[0], float(2.0)) +
                                     pow(tanVelocity.values[1], float(2.0)) +
                                     pow(tanVelocity.values[2], float(2.0)));

        // tanVelocity_mag x 1000 because texture models are in mm/s
        tanVelocity_mag *= 1000.0f;

        // unit vector parallel to direction of motion
        for (int i=0;i<6;i++){
            tanVec.values[i] = tanVelocity.values[i]/tanVelocity_mag;
        }

        // HashAndInterp2() is a function defined in AccSynthHashMatrix.cpp
        // that interpolates the model forces using the current normal force
        // and tangential velocity
        mymatrix.HashAndInterp2(textNum, tanVelocity_mag, forceNorm);

        // unit vector for display of texture force
        for (int i=0;i<6;i++){
            textureVec.values[i] = 0.0f;
        }
        // set texture vector to be binormal to current velocity & normal
        //textureVec.values[0] = tanVec.values[1] * normalVec.values[2] - tanVec.values[2] * normalVec.values[1];
        //textureVec.values[1] = tanVec.values[2] * normalVec.values[0] - tanVec.values[0] * normalVec.values[2];
        //textureVec.values[2] = tanVec.values[0] * normalVec.values[1] - tanVec.values[1] * normalVec.values[0];

        // set texture vector to z-axis
        textureVec.values[2] = 1.0f;

        output = vibrations(); //calculate next output texture vibration (in m/s^2)


        /** Set Texture Forces **/
        // scale output value by effective mass of MLHD flotor & user's hand (~0.50kg)
        for (int i=0;i<6;i++){
            forceTex.values[i] = textureVec.values[i] * output * 0.50f;
        }


        /** Calculate Friction Forces **/
        // friction forces are rendered parallel to the current velocity
        if (tanVelocity_mag < vThresh) // if below velocity threshold, use viscous friction
        {
            //forceFric = -mu_k*forceNorm*slopeFric*tanVelocity;
            for (int i=0;i<6;i++){
                // tanVelocity x1000 to match units as mm/s
                forceFric.values[i] = -mu_k*forceNorm*slopeFric*(tanVelocity.values[i] * 1000.0f);
            }
        }
        else // else Coulomb friction, mu*Fn
        {
            //forceFric = -mu_k*forceNorm*tanVec;
            for (int i=0;i<6;i++){
                forceFric.values[i] = -mu_k*forceNorm*tanVec.values[i];
            }
        }

        // debug stream to terminal
        /*std::cout << "forceNorm= " << forceNorm << " N, output= " << output <<
                     " m*s^-2, tanVel= " << tanVelocity_mag <<" mm/s" <<
                     std::endl;*/


        /** Add normal spring force from "penetrating" surface**/
        // Moved to end of callback to reduce interference with vibration and
        // friction calculations
        // - As would have been set by Omni's UpdateEffectorPosition()
        // - Times 1000 because original OMNI implimentation uses mm for position
        // - STIFFNESS = 0.5 from Constants.h, midpoint of hardnest surfaces OMNI can render
        //   --> max stiffness = 2.31N/mm in vertical, for STIFFNESS == 1.0
        //   --> STIFFNESS = 1.15N/mm to correspond to STIFFNESS == 0.5
        //forceNorm = STIFFNESS * (0.0 - 1000.0*curr_p_z);
        forceNorm = stiffness * (0.0 - 1000.0*curr_p_z);

	//forceNorm = stiffness * (0.0 - 1000.0*curr_p_z) - dampingFactor * (normalVelocity.values[2]*1000.0f);
	// as i increase stiffness, scale dampingFactor by the increase in stiffness, to maintain 
	// the ratio between stiffness/dampingFactor


        /** Display forces based on user input **/
        // output force is resultant sum of normal, texture, and friction forces
        //forceVec = forceNorm*normalVec + forceTex + forceFric;
        for (int i=0;i<6;i++){
            forceVec.values[i] = forceNorm*normalVec.values[i] + forceTex.values[i] + forceFric.values[i];
        }

        if (dispFric){
            if (dispTex){
                // output force is resultant sum of normal, texture, and friction forces
                //forceVec = forceNorm*normalVec + forceTex + forceFric;
                for (int i=0;i<6;i++){
                    forceVec.values[i] = forceNorm*normalVec.values[i] + forceTex.values[i] + forceFric.values[i];
                }
            }
            else{
                // output force is resultant sum of normal and friction forces (no texture)
                //forceVec = forceNorm*normalVec + forceFric;
                for (int i=0;i<6;i++){
                    forceVec.values[i] = forceNorm*normalVec.values[i] + forceFric.values[i];
                }
            }
        }
        else{
            if (dispTex){
                // output force is resultant sum of normal and texture forces (no friction)
                //forceVec = forceNorm*normalVec + forceTex;
                for (int i=0;i<6;i++){
                    forceVec.values[i] = forceNorm*normalVec.values[i] + forceTex.values[i];
                }
            }
            else{
                // output force is resultant sum of normal force only (no friction or texture)
                //forceVec = forceNorm*normalVec;
                for (int i=0;i<6;i++){
                    forceVec.values[i] = forceNorm*normalVec.values[i];
                }
            }
        }

        // set static inter-loop PID gains for X & Y axes
        VecGains.p = 0.0f;
        VecGains.i = 0.0f;
        VecGains.d = 5.0f;
    }
    else{
        /** Flotor is above the surface **/
        // Force to display (forceVec) on flotor is just gravity, so set to zero
        for (int i = 0; i < 6; i++){
            forceVec.values[i] = 0.0f;
        }

        VecGains.p = 0.0f;
        VecGains.i = 0.0f;
        VecGains.d = 0.0f;
    }

    // store new values for filter in next loop
    priorPosition = currPosition;
    oldTime = currTime;


    /** Set force feedback for all three axes **/
    // add gravity back to forceVec to make flotor weightless
    VecGains.ff = forceVec.values[0] + gravityX;
    ml_SetGainVecAxis(dev_hdl, ML_GAINSET_TYPE_NORMAL, ML_AXIS_X, VecGains);

    VecGains.ff = forceVec.values[1] + gravityY;
    ml_SetGainVecAxis(dev_hdl, ML_GAINSET_TYPE_NORMAL, ML_AXIS_Y, VecGains);

    VecGains.ff = forceVec.values[2] + gravityZ;
    if (curr_p_z < 0.0f) {
        // VecGains.d much higher in Z to reduce instability from high material stiffness
        float ratio = stiffness / STIFFNESS_OMNI1_0x;
        if (ratio >= 12.0f) VecGains.d = 95.0f;
        else if (ratio >= 8.0f) VecGains.d = 60.0f;
        else if (ratio >= 4.0f) VecGains.d = 30.0f;
        else VecGains.d = 15.0f;
    }
    ml_SetGainVecAxis(dev_hdl, ML_GAINSET_TYPE_NORMAL, ML_AXIS_Z, VecGains);


    return 0;
}

/******************************************************************************
Penn Toolkit:   Method to handle keyboard input.
******************************************************************************/
void processNormalKeys(unsigned char key, int x, int y) {
	
  GLuint Texture;
  int i;

  switch (key) {
    // close the simulation
    case 27: 			//27 is the escape key
    case 'q':
    case 'Q':
      exit(0);		//close the simulation
      break;

    // + key to scroll up through textures
    case 43:
      if(textNum<NUM_TEX-1)
        textNum = textNum+1;

      // display correct texture group name
      if(textNum>94)
        texGroupNum = 9;
      else if(textNum>92)
        texGroupNum = 8;
      else if(textNum>85)
        texGroupNum = 7;
      else if(textNum>79)
        texGroupNum = 6;
      else if(textNum>73)
        texGroupNum = 5;
      else if(textNum>68)
        texGroupNum = 4;
      else if(textNum>61)
        texGroupNum = 3;
      else if(textNum>31)
        texGroupNum = 2;
      else if(textNum>21)
        texGroupNum = 1;
      else
        texGroupNum = 0;

      Texture = loadBMP_custom(imArray[textNum]); // load the texture image

      glutDisplayFunc(displayFunction); // change display of texture name
      break;

    // - key to scroll down through textures
    case 45:
      if(textNum>0)
        textNum = textNum-1;

      // display correct texture group name
      if(textNum>94)
        texGroupNum = 9;
      else if(textNum>92)
        texGroupNum = 8;
      else if(textNum>85)
        texGroupNum = 7;
      else if(textNum>79)
        texGroupNum = 6;
      else if(textNum>73)
        texGroupNum = 5;
      else if(textNum>68)
        texGroupNum = 4;
      else if(textNum>61)
        texGroupNum = 3;
      else if(textNum>31)
        texGroupNum = 2;
      else if(textNum>21)
        texGroupNum = 1;
      else
        texGroupNum = 0;

      Texture = loadBMP_custom(imArray[textNum]); // load the texture image
      glutDisplayFunc(displayFunction); // change display of texture name
      break;

    // toggle friction
    case 'f':
    case 'F':
      if(dispFric)
        dispFric=false;
      else
        dispFric=true;
      // display current state of friction
      glutDisplayFunc(displayFunction);
      break;

    // toggle texture force
    case 't':
    case 'T':
      if(dispTex)
        dispTex=false;
      else
        dispTex=true;
      // display current state of texture force
      glutDisplayFunc(displayFunction);
      break;

    // toggle stiffnesses
    case 'j':
    case 'J':
      if (stiffness == STIFFNESS_OMNI2_0x) i = 0;
      else if (stiffness == STIFFNESS_OMNI1_5x) i = 1;
      else if (stiffness == STIFFNESS_OMNI1_0x) i = 2;
      else if (stiffness == STIFFNESS_OMNI0_5x) i = 3;
      // if stiffness has been adjusted past these values
      // reset to 2x omni on keypress
      else i = 3; 
      
      if (i == 3) stiffness = STIFFNESS[0];
      else stiffness = STIFFNESS[i+1];

      glutDisplayFunc(displayFunction);
      break;
    
    // increase stiffness past set points
    case 'l':
    case 'L':
      stiffness += STIFFNESS_OMNI0_5x;
      if (stiffness > 40.0f) {
          stiffness = 17.0f * STIFFNESS_OMNI1_0x;
      }
      glutDisplayFunc(displayFunction);
      break;

    // decrease stiffness past set points
    case 'k':
    case 'K':
      stiffness -= STIFFNESS_OMNI0_5x;
      if (stiffness < STIFFNESS_OMNI0_5x) {
          stiffness = STIFFNESS_OMNI0_5x;
      }
      glutDisplayFunc(displayFunction);
      break;

    // display menu in terminal window
    case 'm':
    case 'M':
      printf("\t\tPenn Haptic Texture Toolkit\n");
      printf("Heather Culbertson, Juan Jose Lopez Delgado, and Katherine Kuchenbecker\n\n");
      printf(" Key Command List \n -----------------------------------------\n");
      printf("    + --> Scroll Forward Through Textures\n");
      printf("    - --> Scroll Backward Through Textures\n");
      printf("    F/f --> Toggle Friction ON/OFF\n");
      printf("    T/t --> Toggle Texture ON/OFF\n");
      printf("    J/j --> Toggle Mat'l Stiffness (0.5x - 2.0x OMNI)\n");
      printf("    L/l --> INCREASE Mat'l Stiffness (to 17.0x OMNI)\n");
      printf("    K/k --> DECREASE Mat'l Stiffness\n");
      printf("\n    Jump to Texture Group\n");
      printf("    ----------------------\n");
      printf("    0 --> Paper (22 textures)\n");
      printf("    1 --> Plastic (10 textures)\n");
      printf("    2 --> Fabric (30 textures)\n");
      printf("    3 --> Tile (7 textures)\n");
      printf("    4 --> Carpet (5 textures)\n");
      printf("    5 --> Foam (6 textures)\n");
      printf("    6 --> Metal (6 textures)\n");
      printf("    7 --> Stone (7 textures)\n");
      printf("    8 --> Carbon Fiber (2 textures)\n");
      printf("    9 --> Wood (5 textures)\n");
      break;

    // jump to Paper texture group
    case '0':
      texGroupNum = 0;
      textNum = 0;

      Texture = loadBMP_custom(imArray[textNum]); // load texture image
      glutDisplayFunc(displayFunction); // display current texture group
      break;

    // jump to Plastic texture group
    case '1':
      texGroupNum = 1;
      textNum = 22;

      Texture = loadBMP_custom(imArray[textNum]); // load texture image
      glutDisplayFunc(displayFunction); // display current texture group
      break;

    // jump to Fabric texture group
    case '2':
      texGroupNum = 2;
      textNum = 32;

      Texture = loadBMP_custom(imArray[textNum]); // load texture image
      glutDisplayFunc(displayFunction); // display current texture group
      break;

    // jump to Tile texture group
    case '3':
      texGroupNum = 3;
      textNum = 62;

      Texture = loadBMP_custom(imArray[textNum]); // load texture image
      glutDisplayFunc(displayFunction); // display current texture group
      break;

    // jump to Carpet texture group
    case '4':
      texGroupNum = 4;
      textNum = 69;

      Texture = loadBMP_custom(imArray[textNum]); // load texture image
      glutDisplayFunc(displayFunction); // display current texture group
      break;

    // jump to Foam texture group
    case '5':
      texGroupNum = 5;
      textNum = 74;

      Texture = loadBMP_custom(imArray[textNum]); // load texture image
      glutDisplayFunc(displayFunction); // display current texture group
      break;

    // jump to Metal texture group
    case '6':
      texGroupNum = 6;
      textNum = 80;

      Texture = loadBMP_custom(imArray[textNum]); // load texture image
      glutDisplayFunc(displayFunction); // display current texture group
      break;

    // jump to Stone texture group
    case '7':
      texGroupNum = 7;
      textNum = 86;

      Texture = loadBMP_custom(imArray[textNum]); // load texture image
      glutDisplayFunc(displayFunction); // display current texture group
      break;

    // jump to Carbon Fiber texture group
    case '8':
      texGroupNum = 8;
      textNum = 93;

      Texture = loadBMP_custom(imArray[textNum]); // load texture image
      glutDisplayFunc(displayFunction); // display current texture group
      break;

    // jump to Wood texture group
    case '9':
      texGroupNum = 9;
      textNum = 95;

      Texture = loadBMP_custom(imArray[textNum]); // load texture image
      glutDisplayFunc(displayFunction); // display current texture group
      break;

    // key pressed doesn't do anything
    default:
      printf("You pressed the %c key, for which there is no dedicated action.\n", key);
      break;
    }
}

/*******************************************************************************
Penn Toolkit:   Graphics main loop
*******************************************************************************/
void displayFunction(void)
{

    float posFlotor[3];

    glPushMatrix();
    doGraphicsState();


    // draw surface with texture at z = 0
    displayTextureSurface();

    // draw sphere at most recent flotor position. x1000 because original scaling
    // of displayVisitorSphere() is in mm
    GLUquadricObj* quadObj = gluNewQuadric();
    posFlotor[0] = (float)(1000.0f*curr_Position.values[0]);
    posFlotor[1] = (float)(1000.0f*curr_Position.values[1]);
    posFlotor[2] = (float)(1000.0f*curr_Position.values[2]);
    displayVisitorSphere(quadObj, posFlotor);

    gluDeleteQuadric(quadObj);
    glEnable(GL_LIGHTING);


    // displays text to screen
    printtext(10,760,texArray[textNum],true);  // prints texture name
    if(dispFric)
        printtext(10,735,"Friction ON!",false); // display of friction is on
    else
        printtext(10,735,"Friction OFF!",false); // display of friction is off
    if(dispTex)
        printtext(10,710,"Texture ON!",false); // display of texture is on
    else
        printtext(10,710,"Texture OFF!",false); // display of texture is off

    // prints name of texture group
    if(texGroupNum==0)
        printtext(10,10,"Paper",true);
    else if(texGroupNum==1)
        printtext(10,10,"Plastic",true);
    else if(texGroupNum==2)
        printtext(10,10,"Fabric",true);
    else if(texGroupNum==3)
        printtext(10,10,"Tile",true);
    else if(texGroupNum==4)
        printtext(10,10,"Carpet",true);
    else if(texGroupNum==5)
        printtext(10,10,"Foam",true);
    else if(texGroupNum==6)
        printtext(10,10,"Metal",true);
    else if(texGroupNum==7)
        printtext(10,10,"Stone",true);
    else if(texGroupNum==8)
        printtext(10,10,"Carbon Fiber",true);
    else if(texGroupNum==9)
        printtext(10,10,"Wood",true);

    // display current stiffness value
    if(stiffness == STIFFNESS_OMNI2_0x)
        printtext(550,10,"k = 4.60 N/mm (2.0x OMNI)",false);
    else if (stiffness == STIFFNESS_OMNI1_5x)
        printtext(550,10,"k = 3.45 N/mm (1.5x OMNI)",false);
    else if (stiffness == STIFFNESS_OMNI1_0x)
        printtext(550,10,"k = 2.30 N/mm (1.0x OMNI)",false);
    else if (stiffness == STIFFNESS_OMNI0_5x)
        printtext(550,10,"k = 1.15 N/mm (0.5x OMNI)",false);
    else {
        float ratio = stiffness / STIFFNESS_OMNI1_0x;
        std::ostringstream s1;
        s1 << stiffness; 
        std::ostringstream s2;
        s2 << ratio;
        std::string str = "k = " + s1.str() + " N/mm (" +
                          s2.str() + "x OMNI)";
        printtext(550,10,str,false);
    }

    // display 'm' & 'q' button help
    printtext(550,760,"Press 'm' to display menu",false);
    printtext(550,735,"to terminal",false);
    printtext(550,710,"Press 'q' to quit",false);


    glPopMatrix();
    glutSwapBuffers();

    // trigger the display function again (since we use no idle function)
    glutPostRedisplay();
}


/******************************************************************************
Main Function
******************************************************************************/
int main(int argc, char* argv[])
{

    gettimeofday(&tv0, NULL);
    starttime = tv0.tv_usec;
    printf("Starting Application\n\n");
    printf("\t\tPenn Haptic Texture Toolkit\n");
    printf("Heather Culbertson, Juan Jose Lopez Delgado, and Katherine Kuchenbecker\n\n");
    atexit(exitHandler);

    // initialize the MLHD, measure gravity, and float flotor to origin
    int Initializeflag;
    Initializeflag = InitializeMLHD(Default_IP, &Device_Handle);
    if(Initializeflag) 
        return -1;

    // load texture file
    AccSynthHashMatrix thematrix = generateHashMatrix();
    mymatrix = thematrix;
    printf("Texture matrix created successfully!\n");

    // constrain the flotor
    ml_ConstrainAxis (Device_Handle, ML_AXIS_X_POS, ConstrainXAxisMin, ConstrainXAxisMax);
    ml_ConstrainAxis (Device_Handle, ML_AXIS_Y_POS, ConstrainYAxisMin, ConstrainYAxisMax);
    ml_ConstrainAxis (Device_Handle, ML_AXIS_Z_POS, ConstrainZAxisMin, ConstrainZAxisMax);

    // read the gravity into global variables
    gravityX = Gravity.values[0];
    gravityY = Gravity.values[1];
    gravityZ = Gravity.values[2];


    // initGlut() in helper.cpp to initialize glut visualizations
    //printf("Got to main(): initGlut()\n");
    initGlut(argc, argv);

    //maxWorkSpace is in units mm
    //Low, Left, Back point of device workspace.
    // y-axis is positive at back of MLHD (furthest away from user)
    float LLB[3] = {1000.0f*ConstrainXAxisMin, 1000.0f*ConstrainYAxisMax,
                    1000.0f*ConstrainZAxisMin};

    //Top, Right, Front point of device workspace.
    float TRF[3] = {1000.0f*ConstrainXAxisMax, 1000.0f*ConstrainYAxisMin,
                    1000.0f*ConstrainZAxisMax};

    // initGraphics
    //printf("Got to main(): initGraphics()\n");
    initGraphics(LLB, TRF);

    // print menu to terminal window after initialization
    printf("\t\tPenn Haptic Texture Toolkit\n");
    printf("Heather Culbertson, Juan Jose Lopez Delgado, and Katherine Kuchenbecker\n");
    printf(" Key Command List \n -----------------------------------------\n");
    printf("    + --> Scroll Forward Through Textures\n");
    printf("    - --> Scroll Backward Through Textures\n");
    printf("    F/f --> Toggle Friction ON/OFF\n");
    printf("    T/t --> Toggle Texture ON/OFF\n");
    printf("    J/j --> Toggle Mat'l Stiffness (0.5x - 2.0x OMNI)\n");
    printf("    L/l --> INCREASE Mat'l Stiffness (to 17.0x OMNI)\n");
    printf("    K/k --> DECREASE Mat'l Stiffness\n");
    printf("\n    Jump to Texture Group\n");
    printf("    ----------------------\n");
    printf("    0 --> Paper (22 textures)\n");
    printf("    1 --> Plastic (10 textures)\n");
    printf("    2 --> Fabric (30 textures)\n");
    printf("    3 --> Tile (7 textures)\n");
    printf("    4 --> Carpet (5 textures)\n");
    printf("    5 --> Foam (6 textures)\n");
    printf("    6 --> Metal (6 textures)\n");
    printf("    7 --> Stone (7 textures)\n");
    printf("    8 --> Carbon Fiber (2 textures)\n");
    printf("    9 --> Wood (5 textures)\n");


    //printf("Got to main(): RegisterCallbackTick()");
    if(ML_STATUS_OK != ml_RegisterCallbackTick(Device_Handle, tick_callback_handler)){
        printf("\n\tERROR: Failed to register the callback function\n");
        ml_Land(Device_Handle);
        ml_Disconnect(Device_Handle);
        return -1;
    }


    // enter Glut main loop
    glutMainLoop();


    printf("Done\n");
    return 0;
}


//********* E * O * F *********//

/***********************************************************************************************************************************
COPYRIGHT AND PERMISSION NOTICE
Penn Software The Penn Haptic Texture Toolkit
Copyright (C) 2013 The Trustees of the University of Pennsylvania
All rights reserved.

The Trustees of the University of Pennsylvania (“Penn”) and Heather Culbertson, Juan Jose Lopez Delgado, and Katherine J. Kuchenbecker, the developer (“Developer”) of Penn Software The Penn Haptic Texture Toolkit (“Software”) give recipient (“Recipient”) and Recipient’s Institution (“Institution”) permission to use, copy, and modify the software in source and binary forms, with or without modification for non-profit research purposes only provided that the following conditions are met:

1)	All copies of Software in binary form and/or source code, related documentation and/or other materials provided with the Software must reproduce and retain the above copyright notice, this list of conditions and the following disclaimer.

2)	Recipient shall have the right to create modifications of the Software (“Modifications”) for their internal research and academic purposes only. 

3)	All copies of Modifications in binary form and/or source code and related documentation must reproduce and retain the above copyright notice, this list of conditions and the following disclaimer.

4)	Recipient and Institution shall not distribute Software or Modifications to any third parties without the prior written approval of Penn.

5)	Recipient will provide the Developer with feedback on the use of the Software and Modifications, if any, in their research.  The Developers and Penn are permitted to use any information Recipient provides in making changes to the Software. All feedback, bug reports and technical questions shall be sent to: 

Heather Culbertson, hculb@seas.upenn.edu, +1 215-573-6748
Katherine J. Kuchenbecker, kuchenbe@seas.upenn.edu, +1 215-573-2786

6)	Recipient acknowledges that the Developers, Penn and its licensees may develop modifications to Software that may be substantially similar to Recipient’s modifications of Software, and that the Developers, Penn and its licensees shall not be constrained in any way by Recipient in Penn’s or its licensees’ use or management of such modifications. Recipient acknowledges the right of the Developers and Penn to prepare and publish modifications to Software that may be substantially similar or functionally equivalent to your modifications and improvements, and if Recipient or Institution obtains patent protection for any modification or improvement to Software, Recipient and Institution agree not to allege or enjoin infringement of their patent by the Developers, Penn or any of Penn’s licensees obtaining modifications or improvements to Software from the Penn or the Developers.

7)	Recipient and Developer will acknowledge in their respective publications the contributions made to each other’s research involving or based on the Software. The current citations for Software are:

Heather Culbertson, Juan Jose Lopez Delgado, and Katherine J. Kuchenbecker. One Hundred Data-Driven Haptic Texture Models and Open-Source Methods for Rendering on 3D Objects. In Proc. IEEE Haptics Symposium, February 2014.

8)	Any party desiring a license to use the Software and/or Modifications for commercial purposes shall contact The Center for Technology Transfer at Penn at 215-898-9591.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS, CONTRIBUTORS, AND THE TRUSTEES OF THE UNIVERSITY OF PENNSYLVANIA "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER, CONTRIBUTORS OR THE TRUSTEES OF THE UNIVERSITY OF PENNSYLVANIA BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

************************************************************************************************************************************/
