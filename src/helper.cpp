/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com
	
Edited by Juan Jose Lopez Delgado and Heather Culbertson (2014) for inclusion in the Penn Haptic Texture Toolkit
Edited by Randy Lee (2015) to implement the Texture Toolkit on the MLHD

Module name:
  
  helper.cpp

Description:

  Sets the graphics state. 

*****************************************************************************/

#include "Constants.h"
#include <cstdio>
#include <math.h>
#include <iostream>
#include "shared.h" 
#if defined(WIN32) || defined(linux)
# include <GL/glut.h>
#elif defined(__APPLE__)
# include <GLUT/glut.h>
#endif

extern void Resize(int, int);
extern void displayFunction(void);
extern void handleMenu(int);
extern void handleIdle(void);
extern void handleMouse(int, int, int, int);
extern void processNormalKeys(unsigned char key, int x, int y);
GLuint Texture;

/*****************************************************************************
  GLUT initialization
*****************************************************************************/
void initGlut(int argc, char* argv[])
{
  glutInit(&argc, argv); //init GLUT
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(800,800);
  glutInitWindowPosition(100, 100);
  //glutCreateWindow("Textured Sphere");
  glutCreateWindow("Penn Haptic Texture Toolkit -- MLHD");

  glutReshapeFunc(Resize);
  glutDisplayFunc(displayFunction); //setup GLUT callbacks
  glutMouseFunc(handleMouse);
  glutKeyboardFunc(processNormalKeys);  // register processNormalKeys to deal with keyboard input
  //glutIdleFunc(handleIdle);    // OMNI implementation w/ phantom variables
  glutIdleFunc(NULL);            // MLHD implementation

  glutCreateMenu(handleMenu);
  glutAddMenuEntry("Quit", 0);
  glutAttachMenu(GLUT_RIGHT_BUTTON);
}

/*******************************************************************************
  Load BMP texture 
*******************************************************************************/
GLuint loadBMP_custom(const char * imagepath)
{
// Data read from the header of the BMP file
unsigned char header[54]; // Each BMP file begins by a 54-bytes header
unsigned int dataPos;     // Position in the file where the actual data begins
unsigned int width, height;
unsigned int imageSize;   // = width*height*3

// Debugging
//printf("loading: %s", imagepath);

// Open the file
FILE * file = fopen(imagepath,"rb");

if (!file) {std::cout << "Image could not be opened" << std::endl; return 0;}

if ( fread(header, 1, 54, file)!=54 ){ // If not 54 bytes read : problem
    std::cout << "Not a correct BMP file" << std::endl;
    return false;
}

if ( header[0]!='B' || header[1]!='M' ){
    std::cout << "Not a correct BMP file" << std::endl;
    return 0;
}

// Read ints from the byte array
dataPos    = *(int*)&(header[0x0A]);
imageSize  = *(int*)&(header[0x22]);
width      = *(int*)&(header[0x12]);
height     = *(int*)&(header[0x16]);

// Some BMP files are misformatted, guess missing information
if (imageSize==0)    imageSize=width*height*3; // 3 : one byte for each Red, Green and Blue component
if (dataPos==0)      dataPos=54; // The BMP header is done that way
 	
fread(imData,1,imageSize,file);
 
//Everything is in memory now, the file can be closed
fclose(file);

// Create one OpenGL texture
GLuint textureID;
glGenTextures(1, &textureID);

// "Bind" the newly created texture : all future texture functions will modify this texture
glBindTexture(GL_TEXTURE_2D, textureID);

// Give the image to OpenGL
glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, imData);
glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); //mix with light
/* OMNI Implementation w/ texture wrapping
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
*/
// MLHD plane implementation
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

return textureID;
}

/*******************************************************************************
Print text to screen
*******************************************************************************/
void printtext(int x, int y, std::string String, bool sizeL)
{
  //(x,y) is from the bottom left of the window
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, 800, 0, 800, -1.0f, 1.0f);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glPushAttrib(GL_DEPTH_TEST);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);   //to disable drawing strings with current background color
  glDisable(GL_TEXTURE_2D);
  //glColor3f(255, 255, 255);
  glColor3f(1.0f, 1.0f, 1.0f);
  glRasterPos2i(x,y);

  // two possible font sizes for display
  if (sizeL) {
      // print string one character at a time
      for (int i=0; i<String.size(); i++)
      {
          glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, String[i]);
      }
  }
  else {
      // print string one character at a time
      for (int i=0; i<String.size(); i++)
      {
          glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, String[i]);
      }
  }

  glEnable(GL_LIGHTING);
  glEnable(GL_TEXTURE_2D);
  glPopAttrib();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

/*****************************************************************************
  Use haptic device coordinate space as model for the graphics. Define 
  orthographic projection to fit it

  LLB: Low, Left, Back point of device workspace
  TRF: Top, Right, Front point of device workspace
*****************************************************************************/
void initGraphics(const float LLB[3], const float TRF[3])//initGraphics(const HDdouble LLB[3], const HDdouble TRF[3])
{
  glMatrixMode(GL_PROJECTION); //setup perspective
  glLoadIdentity();

  Texture = loadBMP_custom(imArray[0]);

  glBindTexture(GL_TEXTURE_2D, Texture);

  // Give the image to OpenGL
  glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, 1024, 1024, 0, GL_BGR, GL_UNSIGNED_BYTE, imData);
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); //mix with light
  /* OMNI Implementation w/ texture wrapping
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  */
  // MLHD Implementation for plane
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
 		
  //HDdouble centerScreen[3];
  float centerScreen[3];
  centerScreen[0] = (TRF[0] + LLB[0])/2.0;
  centerScreen[1] = (TRF[1] + LLB[1])/2.0;
  centerScreen[2] = (TRF[2] + LLB[2])/2.0;

  //HDdouble screenDims[3];
  float screenDims[3];
  screenDims[0] = TRF[0] - LLB[0];
  screenDims[1] = TRF[1] - LLB[1];
  screenDims[2] = TRF[2] - LLB[2];

  //HDdouble maxDimXY = (screenDims[0] > screenDims[1] ? screenDims[0]:screenDims[1]);
  //HDdouble maxDim = (maxDimXY > screenDims[2] ? maxDimXY:screenDims[2]);
  float maxDimXY = (screenDims[0] > screenDims[1] ? screenDims[0]:screenDims[1]);
  float maxDim = (maxDimXY > screenDims[2] ? maxDimXY:screenDims[2]);
  maxDim /= 4.0;
  glOrtho(centerScreen[0]-maxDim, centerScreen[0]+maxDim,
	  centerScreen[1]-maxDim, centerScreen[1]+maxDim,
	  centerScreen[2]+maxDim, centerScreen[2]-maxDim);

  glShadeModel(GL_SMOOTH);
  
  // Setup model transformations
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  //glutSwapBuffers();
  
  glClearDepth(1.0);
  glDisable(GL_DEPTH_TEST);

}

/*******************************************************************************
  Setup graphics pipeline, lights, usw.
 *******************************************************************************/
void doGraphicsState()
{
  glMatrixMode(GL_MODELVIEW); // setup model transformations
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
  glEnable(GL_COLOR_MATERIAL);
  glShadeModel(GL_SMOOTH);

  glEnable(GL_LIGHTING);
  glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHT_MODEL_TWO_SIDE);
  glShadeModel(GL_SMOOTH);

  GLfloat lightZeroPosition[] = {10.0, 4.0, 100.0, 0.0};
  GLfloat lightZeroColor[] = {0.6, 0.6, 0.6, 1.0}; /* green-tinted */
  GLfloat lightOnePosition[] = {-1.0, -2.0, -100.0, 0.0};
  GLfloat lightOneColor[] = {0.6, 0.6, 0.6, 1.0}; /* red-tinted */

  GLfloat light_ambient[] = {0.8, 0.8, 0.8, 1.0}; /* White diffuse light. */
  GLfloat light_diffuse[] = {0.0, 0.0, 0.0, 1.0}; /* White diffuse light. */
  GLfloat light_position[] = {0.0, 0.0, 100.0, 1.0}; /* Infinite light loc. */

  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
  glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
  glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
}

/*******************************************************************************
  Draw Slave at given position
*******************************************************************************/
void displayVisitorSphere(GLUquadricObj* quadObj, float position[3])
{
  float scaling = 3.0f/8.0f;    // scaling to match the usable MLHD workspace (in mm)
  glMatrixMode(GL_MODELVIEW);

  glPushMatrix();
  
  // display one sphere to be the cursor and dynamic change
  glTranslatef(position[0]*scaling, position[1]*scaling, position[2]*scaling);
  //glColor4f(0.9, 0.2, 0.2, 1.0);
  if (position[2] <= 0.0f) {
      glColor4f(0.9, 0.2, 0.2, 1.0);    // red sphere when in contact
      gluSphere(quadObj, VISITOR_SPHERE_RADIUS/4.0, 20, 20);
  }
  else {
      //glColor4f(0.9, 0.2, 0.2, 1.0);
      glColor4f(1.0, 1.0, 0.0, 1.0);    // yellow sphere when above surface
      gluSphere(quadObj, (VISITOR_SPHERE_RADIUS/4.0)+(0.05f*position[2]), 20, 20);
  }

  glPopMatrix();

}



/*******************************************************************************
  Draw Master at given position
*******************************************************************************/
void displayTextureSurface()
{
    //glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_NORMAL_MAP);
    //glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_NORMAL_MAP);

    glEnable(GL_TEXTURE_2D);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    //glEnable(GL_TEXTURE_GEN_S);
    //glEnable(GL_TEXTURE_GEN_T);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glTranslatef(0.0f, 0.0f, 0.0f);   // have surface on z = 0 level

    // Draw a saquare on which to wrap texture
    GLfloat textureWidth = 3.0f;
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f);  glVertex2f(-textureWidth, -textureWidth);
    glTexCoord2f(1.0f, 0.0f);  glVertex2f(-textureWidth,  textureWidth);
    glTexCoord2f(1.0f, 1.0f);  glVertex2f( textureWidth,  textureWidth);
    glTexCoord2f(0.0f, 1.0f);  glVertex2f( textureWidth, -textureWidth);
    glEnd();

    glPopMatrix();
    //glDisable(GL_TEXTURE_GEN_S);
    //glDisable(GL_TEXTURE_GEN_T);

}

/*
void displayFixedSphere(GLUquadricObj* quadObj, const double position[3])
{
    glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
    glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);

    glEnable(GL_TEXTURE_2D);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    glEnable(GL_TEXTURE_GEN_S);
    glEnable(GL_TEXTURE_GEN_T);

    glColor4f(1, 1, 1, 1.0);
    glTranslatef(position[0], position[1], position[2]);
    gluSphere(quadObj, FIXED_SPHERE_RADIUS, 20, 20);  
 
    glPopMatrix();
    glDisable(GL_TEXTURE_GEN_S);
    glDisable(GL_TEXTURE_GEN_T);

}
*/

/**************** E * O * F *****************/
