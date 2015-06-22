/***********************************************************************************************************************************
COPYRIGHT AND PERMISSION NOTICE
Penn Software The Penn Haptic Texture Toolkit
Copyright (C) 2013 The Trustees of the University of Pennsylvania
All rights reserved.

See copyright and permission notice at the end of this file.

Report bugs to Heather Culbertson (hculb@seas.upenn.edu, +1 215-573-6748) or Katherine J. Kuchenbecker (kuchenbe@seas.upenn.edu, +1 215-573-2786)

This code is based on the original TexturePad haptic rendering system designed by Joseph M. Romano.
Edited by Randy Lee (2015) to implement the Penn Haptic Texture Toolkit on the MLHD
************************************************************************************************************************************/


/*********************************************************************************
This function reads and parses and specified XML model file. The model information
is stored in a hash table for access by other files.
*********************************************************************************/



#include "AccSynthHashMatrix.h"
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include "shared.h"
#include "pugixml.hpp"
#include <string.h>
#include <dirent.h>
#include <vector>
#include <errno.h>

//#include "sharedMemory.h"
using namespace std;

AccSynthHashMatrix generateHashMatrix()
{

AccSynthHashMatrix tempmatrix(NUM_TEX);

float allSpeeds[200]; //allocate memory for arrays to store lists of speeds and forces
float allForces[200];
int DT1[200]; //allocate memory for arrays to store lists of Delaunay triangulation vertices
int DT2[200];
int DT3[200];
float modLSF[MAX_COEFF]; //allocate memory for arrays to store AR and MA LSFs
float modMALSF[MAX_MACOEFF];

int numMod;
int numTri;
float maxSpeed;
float maxForce;
float mu;

int numCoeff;
int numMACoeff;
float variance;
float gain;
float modSpeed;
float modForce;

string filePath;
string imFilename;

//Find filepath of executable
char result[100];
  ssize_t count = readlink( "/proc/self/exe", result,100);
filePath = std::string( result, (count > 0) ? count : 0 );
string::size_type pos = string(result).find_last_of( "\\/" );

//Find filepath of XML model files
string baseFilename = filePath.substr(0,pos) + "/XML/Models_";
string myFilename;

// Debugging
//printf("in generateHashMatrix:\nfilePath %s,\n xml fileName: %s",
//       filePath, myFilename);

/********* Fill array with known texture names ***********/
// divide textures by group

//Paper
strcpy(texArray[0],"Book");
strcpy(texArray[1],"Bubble Envelope");
strcpy(texArray[2],"Cardboard");
strcpy(texArray[3],"Coffee Filter");
strcpy(texArray[4],"Dot Paper");
strcpy(texArray[5],"Folder");
strcpy(texArray[6],"Gift Box");
strcpy(texArray[7],"Glitter Paper");
strcpy(texArray[8],"Greeting Card");
strcpy(texArray[9],"Masking Tape");
strcpy(texArray[10],"Paper Bag");
strcpy(texArray[11],"Paper Plate 1");
strcpy(texArray[12],"Paper Plate 2");
strcpy(texArray[13],"Playing Card");
strcpy(texArray[14],"Resume Paper");
strcpy(texArray[15],"Sandpaper 100");
strcpy(texArray[16],"Sandpaper 220");
strcpy(texArray[17],"Sandpaper 320");
strcpy(texArray[18],"Sandpaper Aluminum Oxide");
strcpy(texArray[19],"Textured Paper");
strcpy(texArray[20],"Tissue Paper");
strcpy(texArray[21],"Wax Paper");

//Plastic
strcpy(texArray[22],"ABS Plastic");
strcpy(texArray[23],"Binder");
strcpy(texArray[24],"Candle");
strcpy(texArray[25],"File Portfolio");
strcpy(texArray[26],"Frosted Acrylic");
strcpy(texArray[27],"Nitrile Glove");
strcpy(texArray[28],"Plastic Mesh 1");
strcpy(texArray[29],"Plastic Mesh 2");
strcpy(texArray[30],"Tarp");
strcpy(texArray[31],"Wavy Acrylic");

//Fabric
strcpy(texArray[32],"Athletic Shirt");
strcpy(texArray[33],"Blanket");
strcpy(texArray[34],"CD Sleeve");
strcpy(texArray[35],"Canvas 1");
strcpy(texArray[36],"Canvas 2");
strcpy(texArray[37],"Canvas 3");
strcpy(texArray[38],"Cotton");
strcpy(texArray[39],"Denim");
strcpy(texArray[40],"Felt");
strcpy(texArray[41],"Flannel");
strcpy(texArray[42],"Fleece");
strcpy(texArray[43],"Leather 1 Back");
strcpy(texArray[44],"Leather 1 Front");
strcpy(texArray[45],"Leather 2 Back");
strcpy(texArray[46],"Leather 2 Front");
strcpy(texArray[47],"Microfiber Cloth");
strcpy(texArray[48],"Nylon Bag");
strcpy(texArray[49],"Nylon Mesh");
strcpy(texArray[50],"Pleather");
strcpy(texArray[51],"Portfolio Cover");
strcpy(texArray[52],"Silk 1");
strcpy(texArray[53],"Silk 2");
strcpy(texArray[54],"Textured Cloth");
strcpy(texArray[55],"Towel");
strcpy(texArray[56],"Velcro Hooks");
strcpy(texArray[57],"Velcro Loops");
strcpy(texArray[58],"Velvet");
strcpy(texArray[59],"Vinyl 1");
strcpy(texArray[60],"Vinyl 2");
strcpy(texArray[61],"Whiteboard Eraser");

//Tile
strcpy(texArray[62],"Floortile 1");
strcpy(texArray[63],"Floortile 2");
strcpy(texArray[64],"Floortile 3");
strcpy(texArray[65],"Floortile 4");
strcpy(texArray[66],"Floortile 5");
strcpy(texArray[67],"Floortile 6");
strcpy(texArray[68],"Floortile 7");

//Carpet
strcpy(texArray[69],"Artificial Grass");
strcpy(texArray[70],"Carpet 1");
strcpy(texArray[71],"Carpet 2");
strcpy(texArray[72],"Carpet 3");
strcpy(texArray[73],"Carpet 4");

//Foam
strcpy(texArray[74],"EPDM Foam");
strcpy(texArray[75],"Pink Foam");
strcpy(texArray[76],"Polyethylene Foam");
strcpy(texArray[77],"Scouring Pad");
strcpy(texArray[78],"Styrofoam");
strcpy(texArray[79],"Textured Rubber");

//Metal
strcpy(texArray[80],"Aluminum Foil");
strcpy(texArray[81],"Aluminum");
strcpy(texArray[82],"Metal Mesh");
strcpy(texArray[83],"Metal Shelving");
strcpy(texArray[84],"Textured Metal");
strcpy(texArray[85],"Whiteboard");

//Stone
strcpy(texArray[86],"Brick 1");
strcpy(texArray[87],"Brick 2");
strcpy(texArray[88],"Ceramic");
strcpy(texArray[89],"Painted Brick");
strcpy(texArray[90],"Stone Tile 1");
strcpy(texArray[91],"Stone Tile 2");
strcpy(texArray[92],"Terra Cotta");

//Carbon Fiber
strcpy(texArray[93],"Carbon Fiber");
strcpy(texArray[94],"Resin Carbon Fiber");

//Wood
strcpy(texArray[95],"Cork");
strcpy(texArray[96],"MDF");
strcpy(texArray[97],"Painted Wood");
strcpy(texArray[98],"Stained Wood");
strcpy(texArray[99],"Wood");

// Loop through all textures
for (int numSurf=0;numSurf<NUM_TEX;numSurf++)
{
	pugi::xml_document doc;

	myFilename = baseFilename + texArray[numSurf] + ".xml"; // get full filename of model file
	
	pugi::xml_parse_result result = doc.load_file(myFilename.c_str()); // load model file

	pugi::xml_node modelSet = doc.child("modelSet");

	mu = atof(modelSet.child_value("mu")); //friction coefficient

	stringstream ss;
	ss << modelSet.child_value("renderPicture"); //picture for display in rendering
    imFilename = "." + ss.str(); //convert image name to string, edited to conform to ../images/ location
	//imFilename = ss.str();
	strcpy(imArray[numSurf],imFilename.c_str());

	numMod = atoi(modelSet.child_value("numMod")); //number of models

	numTri = atoi(modelSet.child_value("numTri")); //number of triangles in Delaunay triangulation

	maxSpeed = atof(modelSet.child_value("maxSpeed")); //maximum modeled speed

	maxForce = atof(modelSet.child_value("maxForce")); //maximum modeled force

	//list of all model speeds
	int count = 0;
	for (pugi::xml_node speedList = modelSet.child("speedList").child("value"); speedList; speedList = speedList.next_sibling("value"))
	{
	    allSpeeds[count] = atof(speedList.child_value());
	    count++;
	}

	//list of all model forces
	count = 0;
	for (pugi::xml_node forceList = modelSet.child("forceList").child("value"); forceList; forceList = forceList.next_sibling("value"))
	{
	    allForces[count] = atof(forceList.child_value());
	    count++;
	}

	//list of all triangles in Delaunay triangulation
	count = 0;
	for (pugi::xml_node tri = modelSet.child("tri"); tri; tri = tri.next_sibling("tri"))
	{
	    pugi::xml_node triChild = tri.child("value");
	    DT1[count] = atoi(triChild.child_value());
	    DT2[count] = atoi(triChild.next_sibling("value").child_value());
	    DT3[count] = atoi(triChild.next_sibling("value").next_sibling("value").child_value());
	  count++;
	}


	numCoeff = atoi(modelSet.child_value("numARCoeff")); //number of AR coefficients

	numMACoeff = atoi(modelSet.child_value("numMACoeff")); //number of MA coefficients
	if (numMACoeff == 0)
	    isARMA = false;
	else
	    isARMA = true;

	//create a hash table for this surface
	tempmatrix.AddTable(numSurf,numMod,allSpeeds,allForces);

	//for each model in the file
	for (pugi::xml_node model = modelSet.child("model"); model; model = model.next_sibling("model"))
	{
	    //read all AR LSFs
	    count = 0;
	    pugi::xml_node ARlsf = model.child("ARlsf");
	    for (pugi::xml_node ARval = ARlsf.child("value"); ARval; ARval = ARval.next_sibling("value"))
	    {
	     modLSF[count] = atof(ARval.child_value());
	     count++;
	    }
	    
	   //read all MA LSFs (if needed)
	    if(isARMA)
	    {
	     count = 0;
	     pugi::xml_node MAlsf = model.child("MAlsf");
	     for (pugi::xml_node MAval = MAlsf.child("value"); MAval; MAval = MAval.next_sibling("value"))
	     {
	      modMALSF[count] = atof(MAval.child_value());
	      count++;
	     }
	     gain = atof(model.child("gain").child_value());
	    }

	    variance = atof(model.child("var").child_value()); //model variance
	    modSpeed = atof(model.child("speedMod").child_value()); //model speed
	    modForce = atof(model.child("forceMod").child_value()); //model force

	    //create a hash entry for each model
	    if(isARMA){
	    AccSynthHashEntry HashEntry(numSurf,modForce,modSpeed,modLSF,modMALSF,variance,gain,numCoeff,numMACoeff,numTri,numMod,DT1,DT2,DT3,maxSpeed,maxForce,mu);
	    tempmatrix.AddEntry(HashEntry,numMod,allSpeeds,allForces);}
	    else{
	    AccSynthHashEntry HashEntry(numSurf,modForce,modSpeed,modLSF,variance,numCoeff,numTri,numMod,DT1,DT2,DT3,maxSpeed,maxForce,mu);
	    tempmatrix.AddEntry(HashEntry,numMod,allSpeeds,allForces);}
	}

} // end loop through all textures

return tempmatrix;
}

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
