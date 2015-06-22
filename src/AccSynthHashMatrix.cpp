/***********************************************************************************************************************************
COPYRIGHT AND PERMISSION NOTICE
Penn Software The Penn Haptic Texture Toolkit
Copyright (C) 2013 The Trustees of the University of Pennsylvania
All rights reserved.

See copyright and permission notice at the end of this file.

Report bugs to Heather Culbertson (hculb@seas.upenn.edu, +1 215-573-6748) or Katherine J. Kuchenbecker (kuchenbe@seas.upenn.edu, +1 215-573-2786)

This code is based on the original TexturePad haptic rendering system designed by Joseph M. Romano.
************************************************************************************************************************************/

#include "AccSynthHashMatrix.h"
#include "shared.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <complex>

using namespace std;
/******************************************************************************
  AccSynthHashEntry default ctor
******************************************************************************/
AccSynthHashEntry::AccSynthHashEntry()
{
    dummy = true;

    variance = 0.0;
    maxSpeed = 0.0;
    maxForce = 0.0;
}

/******************************************************************************
  AccSynthHashEntry constructor for AR models
******************************************************************************/
AccSynthHashEntry::AccSynthHashEntry(int mysurfType, float myforce, float myspeed, float *myLSF, float myVariance, int myCoeffNum, int myNumTri, int myNumMod, int *myDT1, int *myDT2, int *myDT3, float myMaxSpeed, float myMaxForce, float myMu)
{
    mu = myMu; //kinetic friction coefficient
    force = myforce; //model force
    speed = myspeed; //model speed
    numCoeff = myCoeffNum; //number of AR coefficients
    numTri = myNumTri; //number of triangles in Delaunay triangulation
    numMod = myNumMod; //number of models
    surfType = mysurfType; //surface number (0-99)
    variance = myVariance; //model variance
    maxSpeed = myMaxSpeed; //maximum modeled speed
    maxForce = myMaxForce; //maximum modeled force
    dummy = false;

    filtLSF = new float [numCoeff]; //allocates memory for array of AR LSFs

    DT1 = new int [numTri]; //allocates memory for arrays to store model vertex information for Delaunay triangulation
    DT2 = new int [numTri];
    DT3 = new int [numTri];

    //Get the AR line spectral frequencies for the model
    for(int i =0; i< numCoeff; i++){
	filtLSF[i] = myLSF[i];
    }

    //Get the Delaunay triangulation information, loops through all triangles
    for(int i =0; i< numTri; i++){
        DT1[i] = myDT1[i]; //Each of these represents the one vertex of a triangle
	DT2[i] = myDT2[i];
	DT3[i] = myDT3[i];
    }
}

/******************************************************************************
  AccSynthHashEntry constructor for ARMA models
******************************************************************************/
AccSynthHashEntry::AccSynthHashEntry(int mysurfType, float myforce, float myspeed, float *myLSF, float *myMALSF, float myVariance, float myGain, int myCoeffNum, int myMACoeffNum, int myNumTri, int myNumMod, int *myDT1, int *myDT2, int *myDT3, float myMaxSpeed, float myMaxForce, float myMu)
{
    mu = myMu; //kinetic friction coefficient
    force = myforce; //model force
    speed = myspeed; //model speed
    gain = myGain; //gain of MA coefficients
    numCoeff = myCoeffNum; //number of AR coefficients
    numMACoeff = myMACoeffNum; //number of MA coefficients
    numTri = myNumTri; //number of triangles in Delaunay triangulation
    numMod = myNumMod; //number of models
    surfType = mysurfType; //surface number (0-99)
    variance = myVariance; //model variance
    maxSpeed = myMaxSpeed; //maximum modeled speed
    maxForce = myMaxForce; // maximum modeled force
    dummy = false;

    filtLSF = new float [numCoeff]; //allocates memory for array of AR LSFs
    filtMALSF = new float [numMACoeff]; //allocates memory for array of MA LSFs

    DT1 = new int [numTri]; //allocates memory for arrays to store model vertex information for Delaunay triangulation
    DT2 = new int [numTri];
    DT3 = new int [numTri];

    //Get the AR line spectral frequencies for the model
    for(int i =0; i< numCoeff; i++){
	filtLSF[i] = myLSF[i];
    }

    //Get the MA line spectral frequencies for the model
    for(int i =0; i< numMACoeff; i++){
        filtMALSF[i] = myMALSF[i];
    }
   
    //Get the Delaunay triangulation information, loops through all triangles
    for(int i =0; i< numTri; i++){
        DT1[i] = myDT1[i]; //Each of these represents the one vertex of a triangle
	DT2[i] = myDT2[i];
	DT3[i] = myDT3[i];
    }


}

/******************************************************************************
  AccSynthHashEntry dummy constructor for when no coefficients/variance data exists
******************************************************************************/
AccSynthHashEntry::AccSynthHashEntry(int mysurfType, float myforce, float myspeed)
{
    surfType = mysurfType; //surface number (0-99)
    force = myforce; //model force
    speed = myspeed; //model speed
    dummy = true;

    filtLSF = new float [1]; //allocate memory for AR LSFs
    DT1 = new int [1]; //allocates memory for arrays to store model vertex information for Delaunay triangulation
    DT2 = new int [1];
    DT3 = new int [1];

    filtLSF[0] = 0; //dummy LSFs
    DT1[0] = 1;  //dummy triangle
    DT2[0] = 1;
    DT3[0] = 1;

    variance = 0.0; //dummy variance
    maxSpeed = 0.0; //dummy maximum modeled speed
    maxForce = 0.0; //dummy maximum modeled force
}



/******************************************************************************
  AccSynthHashEntry default destructor
******************************************************************************/
AccSynthHashEntry::~AccSynthHashEntry()
{}


/******************************************************************************
  AccSynthHashTable default ctor
******************************************************************************/
AccSynthHashTable::AccSynthHashTable()
{}


/******************************************************************************
  AccSynthHashTable Constructor for creating blank tables with equally spaced force/velocity arrays
******************************************************************************/
AccSynthHashTable::AccSynthHashTable(int surfNum, int numMod, float speedMod[], float forceMod[])
{
    // generate the memory for an appropriately sized table

        hashMap = new AccSynthHashEntry[numMod];

    // populate our table with dummy force and speed entries
    for(int k=0; k<numMod; k++)
    {
	(hashMap[k]).surfType = surfNum;
	(hashMap[k]).force =forceMod[k];
	(hashMap[k]).speed =speedMod[k];
    }

}

/******************************************************************************
  AccSynthHashTable default destructor
******************************************************************************/
AccSynthHashTable::~AccSynthHashTable()
{
    //NOTE! We don't need to delete the memory used here since we take care of it in the AccSynthHashMatrix destructor.
    //NOTE! It would have been nice to delete it here, but the delete keyword does not handle nested class deletes nicely enough to trust it
}

/******************************************************************************
  Interpolation between models
******************************************************************************/
// method to hash some values and change the appropriate shared memory to reflect this query
void AccSynthHashTable::HashAndInterp2(float interpSpeed, float interpForce)
{
	/******************************************************************************
        Visibility-Walk Collision Detection with Delaunay triangulation (to determine
	which 3 models to interpolate between)
        
        This code is based upon the article:
            O. Devillers, S. Pion, and M. Teillaud. Walking in a triangulation.
	In Proc. ACM Seventeenth Annual Symposium on Computational Geometry, pages
	106-114, New York, NY, USA, 2001. ACM.
        ******************************************************************************/

	//Declaration of variables needed in visibility-walk algorithm
        float px; //x-coordinate (speed) of query point
	float py; //y-coordinate (force) of query point
	float VeX; //x-coordinate of vector describing edge of triangle
	float VeY; //y-coordinate of vector describing edge of triangle
	float ReX; 
	float ReY;
	float tXmid; //x-coordinate of triangle midpoint
	float tYmid; //y-coordinate of triangle midpoint
	float VpX;
	float VpY;
	float RpX;
	float RpY;
	float num;
	float den;
	float RintX;
	float RintY;
	float dotP1;
	float dotP2;
	float length2e;
	float length2p;
	float BC1; //Barycentric coordinate for model at vertex 1
	float BC2; //Barycentric coordinate for model at vertex 2
	float BC3; //Barycentric coordinate for model at vertex 3
	int ei; //current edge of triangle being checked
	int count;
	int t[3]; //model vertices of current triangle
	int modn; //first model on edge being checked
	int modn1; //second model on edge being checked
	int mod1;
	int mod2;
	int cmod;
	int dt[3]; //model vertices of current triangle
	bool bEnd = false; //boolean flag to tell code to stop searching for triangle??????
	bool foundTri = false; //boolean flag if model is inside current triangle
	bool foundM = false; //boolean flag if for new triangle to check

        //Get information about the surface and set global variables
	int numCoeff = hashMap[0].numCoeff; //number of AR coefficients
	int numMACoeff = hashMap[0].numMACoeff; //number of MA coefficients
	coeffNum = numCoeff; // set global variable for number of AR coefficients
	MAcoeffNum = numMACoeff; //set global variable for number of MA coefficients
 	float mu = hashMap[0].mu; //kinetic friction 
	mu_k = mu; //set global variable for mu

	if (interpSpeed>hashMap[0].maxSpeed) //if user's speed is greater than maximum modeled speed, saturate
		interpSpeed = hashMap[0].maxSpeed-0.01; //must be slightly inside convex hull because of rounding errors
	else if (interpSpeed< 0.05) //set speed to essentially zero for very slow speeds
		interpSpeed = 0.0001;

	if (interpForce>hashMap[0].maxForce) //if user's force is greater than maximum modeled force, saturate
		interpForce = hashMap[0].maxForce-0.01; //must be slightly inside conves hull because of rounding errors
	else if	(interpForce<0.05) //set force to essentially zero for very small force
		interpForce = 0.0001;

	px = interpSpeed; //user's current speed
	py = interpForce; //user's current force

	int qi = 15; // Initial point in Delaunay Triangulation to check

	ei = 0; //start searching with edge 0 of triangle

	count = 1; 

	while (!bEnd)
	{
		t[0] = hashMap[0].DT1[qi]; //Model numbers for triangle t with index qi, starting at 1
		t[1] = hashMap[0].DT2[qi];
		t[2] = hashMap[0].DT3[qi];
		modn = t[ei]; //first model on edge being checked
		if (ei<2) //second model on edge being checked
			modn1 = t[ei+1];
		else
			modn1 = t[0];

                //form vector describing edge of triangle
		VeX = hashMap[modn1-1].speed-hashMap[modn-1].speed;
		VeY = hashMap[modn1-1].force-hashMap[modn-1].force;
		ReX = hashMap[modn-1].speed;
		ReY = hashMap[modn-1].force;

                //calculate midpoint of triangle
		tXmid = (hashMap[t[0]-1].speed + hashMap[t[1]-1].speed + hashMap[t[2]-1].speed)/3;
		tYmid = (hashMap[t[0]-1].force + hashMap[t[1]-1].force + hashMap[t[2]-1].force)/3;

                //form vector from query point to midpoint of triangle T
		VpX = tXmid - px;
		VpY = tYmid - py;
		RpX = px;
		RpY = py;

                //Find the point of intersection between line(Vp) and edge (assuming infinite edge length)
		num = (VpX*pow(VeY,2) - VeX*VpY*VeY)*(ReX-RpX) - (VpX*VeX*VeY - pow(VeX,2)*VpY)*(ReY-RpY);
		den = pow((VpX*VeY - VeX*VpY),2);
		RintX = RpX + num/den*VpX; 
		RintY = RpY + num/den*VpY;

                //Check if the point of intersection lies on line segment of triangle edge
		dotP1 = (RintX-ReX)*VeX + (RintY-ReY)*VeY;
		dotP2 = (RintX-RpX)*VpX + (RintY-RpY)*VpY;
		length2e = pow(VeX,2) + pow(VeY,2);
		length2p = pow(VpX,2) + pow(VpY,2);

		if (RpX == RintX && RpY == RintY) //if query point lies on edge
			foundTri = true;	
		else if (dotP1>=0 && dotP1<length2e && dotP2>=0 && dotP2<length2p) //if p vector goes through edge ei
		{
			cmod = 0;
			foundM = false;
			count = 1;
			while (!foundM) //find the triangle adjacent to edge ei
			{
				dt[0] = hashMap[0].DT1[cmod];
				dt[1] = hashMap[0].DT2[cmod];
				dt[2] = hashMap[0].DT3[cmod];
				for(mod1=0;mod1<=2;mod1++)
				{
				    if (dt[mod1]==modn && cmod!=qi)
				    {
			                for (mod2=0;mod2<=2;mod2++)
					{
			                    if (dt[mod2]==modn1)
					    {
				                qi = cmod; //new triangle to check
					        foundM = true;
					    }
				        }
				   }
				}
				cmod++;
				
			}
			
		}
		else if (ei<2 && count<3) //if p vector does NOT go through edge ei, check next edge
		{
			ei = ei+1;
			count = count+1;
		}
		else if (ei==2 && count<3)
		{
			ei = 0;
			count = count+1;
		}
		else //if p vector does NOT go through any of the edges
			foundTri = true;
		
		//when triangle is found, calculate Barycentric coordinates
		if (foundTri)
		{
			BC1 = ((hashMap[t[1]-1].force-hashMap[t[2]-1].force)*(px-hashMap[t[2]-1].speed) + (hashMap[t[2]-1].speed-hashMap[t[1]-1].speed)*(py-hashMap[t[2]-1].force))/((hashMap[t[1]-1].force-hashMap[t[2]-1].force)*(hashMap[t[0]-1].speed-hashMap[t[2]-1].speed) + (hashMap[t[2]-1].speed-hashMap[t[1]-1].speed)*(hashMap[t[0]-1].force-hashMap[t[2]-1].force));
			BC2 = ((hashMap[t[2]-1].force-hashMap[t[0]-1].force)*(px-hashMap[t[2]-1].speed) + (hashMap[t[0]-1].speed-hashMap[t[2]-1].speed)*(py-hashMap[t[2]-1].force))/((hashMap[t[1]-1].force-hashMap[t[2]-1].force)*(hashMap[t[0]-1].speed-hashMap[t[2]-1].speed) + (hashMap[t[2]-1].speed-hashMap[t[1]-1].speed)*(hashMap[t[0]-1].force-hashMap[t[2]-1].force));
			BC3 = 1.0 - BC1 - BC2;
                        if (BC1<0.0 || BC2<0.0 || BC3<0.0){
                            printf("Speed: %f     Force: %f\n",px,py);
                            printf("BC1: %f      BC2: %f     BC3: %f     \n",BC1,BC2,BC3);}
                        
                        if (BC3 < 0.0){
                            BC3 = 0.0;
                            printf("Unstable!\n");
                        }
			bEnd = true;
		}
	}

        /******************************************************************************
        Interpolate AR Line Spectral Frequencies and convert to coefficients
        ******************************************************************************/
	// interpolate the AR LINE SPECTRAL FREQUENCIES
	for(int i =0; i< numCoeff; i++)
	{
	filtLSF[i] =((hashMap[t[0]-1].filtLSF[i]) * BC1) + ((hashMap[t[1]-1].filtLSF[i]) * BC2) + ((hashMap[t[2]-1].filtLSF[i]) * BC3);
	
	}
	
	// Turning Line spectral frequencies into coefficients
        complex<float> pAR[MAX_COEFF]; //roots of P
        complex<float> rQ[MAX_COEFF]; //roots of Q
        complex<float> Q[MAX_COEFF+1]; 
        complex<float> P[MAX_COEFF+1];
        complex<float> Q1[MAX_COEFF+2]; //sum filter
        complex<float> P1[MAX_COEFF+2]; //difference filter
        float AR[MAX_COEFF+1];
        complex<float> rP[MAX_COEFF];

       for(int i =0; i< numCoeff; i++){

            complex<float> mycomplex (0,filtLSF[i]);
            pAR[i]=exp(mycomplex);// e^i*lsf

            if ( i % 2 == 0 ){ // separate the odd index results from the even index results
                rQ[i/2]=pAR[i];
            }
            else{
                rP[(i-1)/2]=pAR[i];
            }
        }

        //if even number of coefficients
	if ( numCoeff % 2 == 0 ){
            for (int i=numCoeff/2;i<numCoeff;i++){ 
                rQ[i]=conj(rQ[i-numCoeff/2]);//add the conjugates of the values to the end of the lists
                rP[i]=conj(rP[i-numCoeff/2]);
            }
            P[0]=1;//P and Q are vectors of 0, starting with a 1
            Q[0]=1;
            for (int i=1;i<=numCoeff;i++){
                Q[i]=0;
                P[i]=0;
            }

            //Form the polynomials P and Q, these should be real
            for (int kQ=0;kQ<numCoeff;kQ++){
                for (int i=kQ+1; i>=1;i--){
                    Q[i] = Q[i]-rQ[kQ]*Q[i-1];
                }
            }
            for (int kP=0;kP<numCoeff;kP++){
                for (int i=kP+1; i>=1;i--){
                    P[i] = P[i]-rP[kP]*P[i-1];
                }
            }

            float vp[2];
            vp[0]=1;
            vp[1]=-1;
            int mp=numCoeff+1;
            int np=2;
            //form difference filter by including root at z=1
            //P1 = conv(P,[1 -1])
            for (int kp=1; kp<mp+np;kp++){
                int j1p=max(1,kp+1-np);
                int j2p=min(kp,mp);
                P1[kp-1]=0;
                for (int jp=j1p; jp<=j2p;jp++){ 
                    P1[kp-1]=P1[kp-1]+P[jp-1]*vp[kp-jp];
                }
            }

            float vq[2];
            vq[0]=1;
            vq[1]=1;
            int mq=mp;
            int nq=np;
            //form sum filter by including root at z=-1
            //Q1 = conv(Q,[1 1])
            for (int kq=1; kq<mq+nq;kq++){
                int j1q=max(1,kq+1-nq);
                int j2q=min(kq,mq);
                Q1[kq-1]=0;
                for (int jq=j1q; jq<=j2q;jq++){ 
                    Q1[kq-1]=Q1[kq-1]+Q[jq-1]*vq[kq-jq];
                }
            }

            //Average the real values for P and Q
            for (int i=0;i<numCoeff+1;i++){
                AR[i]=(P1[i].real()+Q1[i].real())/2;
            }
        }//end even number of coefficients case

        //odd number of coefficients... same thing, but shifted
        else {
            //add the conjugates of the values to the end of the lists
            for (int i=(numCoeff+1)/2;i<numCoeff+1;i++){
                rQ[i]=conj(rQ[i-(numCoeff+1)/2]);
            }
            for (int i=(numCoeff-1)/2;i<numCoeff-1;i++){
                rP[i]=conj(rP[i-(numCoeff-1)/2]);
            }
            P[0]=1; //P and Q are vectors of 0, starting with a 1
            Q[0]=1;
            for (int i=1;i<=numCoeff+1;i++){
                Q[i]=0;
            }

            //Form the polynomials P and Q, these should be real
            for (int i=1;i<=numCoeff-1;i++){
                P[i]=0;
            }

            for (int kQ=0;kQ<numCoeff+1;kQ++){// the order in which it is looped matters
                for (int i=kQ+1; i>=1;i--){
                    Q[i] = Q[i]-rQ[kQ]*Q[i-1];
                }
            }

            for (int kP=0;kP<numCoeff-1;kP++){
                for (int i=kP+1; i>=1;i--){
                    P[i] = P[i]-rP[kP]*P[i-1];
                }
            }

            float v[3];
            v[0]=1;
            v[1]=0;
            v[2]=-1;
            int m=numCoeff;
            int n=3;
            //form difference filter by including root at z=+1 and z=-1
            //P1 = conv(P,[1 0 -1])
            for (int k=1; k<m+n; k++){
                int j1= max(1,k+1-n);
                int j2= min(k,m);
                P1[k-1]=0;
                for (int j=j1; j<=j2;j++){
                    P1[k-1]=P1[k-1]+P[j-1]*v[k-j];
                }
            }
            //Average the real values for P and Q
            for (int i=0;i<numCoeff+1;i++){
                AR[i]=(P1[i].real()+Q[i].real())/2;
            }
        }// end of odd number of coefficients

        //Update appropriate buffer with AR coefficients
        for(int i=1; i<=numCoeff; i++){
            if (SynthesisFlag_Buffer1){
                filtCoeff_buf2[i-1]=AR[i];
            }
            else{
                filtCoeff_buf1[i-1]=AR[i];
            }
        }

        /******************************************************************************
        Interpolate MA Line Spectral Frequencies and convert to coefficients (if needed)
        ******************************************************************************/
	if(isARMA){
		// interpolate the MA LINE SPECTRAL FREQUENCIES
    		for(int i =0; i< numMACoeff; i++)
	    	{
	    		filtMALSF[i] =((hashMap[t[0]-1].filtMALSF[i]) * BC1) + ((hashMap[t[1]-1].filtMALSF[i]) * BC2) + ((hashMap[t[2]-1].filtMALSF[i]) * BC3);
	
	    	}
		//begin transforming MA LSFs to coefficients
		complex<float> pMA[MAX_MACOEFF]; //roots of P
		complex<float> rQ[MAX_MACOEFF]; //roots of Q
		complex<float> Q[MAX_MACOEFF+1];
		complex<float> P[MAX_MACOEFF+1];
		complex<float> Q1[MAX_MACOEFF+2]; //sum filter
		complex<float> P1[MAX_MACOEFF+2]; //difference filter
		float MA[MAX_MACOEFF+1];
		complex<float> rP[MAX_MACOEFF];

	       for(int i =0; i< numMACoeff; i++){

		    complex<float> mycomplex (0,filtMALSF[i]);
		    pMA[i]=exp(mycomplex);// e^i*lsf


		    if ( i % 2 == 0 ){ // separate the odd index results from the even index results
		        rQ[i/2]=pMA[i];
		    }
		    else{
		        rP[(i-1)/2]=pMA[i];
		    }
		}

		if ( numMACoeff % 2 == 0 ){ // if even number of coefficients
		    for (int i=numMACoeff/2;i<numMACoeff;i++){ 
		        rQ[i]=conj(rQ[i-numMACoeff/2]);//add the conjugates of the values to the end of the lists
		        rP[i]=conj(rP[i-numMACoeff/2]);
		    }
		    P[0]=1;// P and Q are vectors of 0, starting with a 1
		    Q[0]=1;
		    for (int i=1;i<=numMACoeff;i++){
		        Q[i]=0;
		        P[i]=0;
		    }

                    //Form the polynomials P and Q, these should be real
		    for (int kQ=0;kQ<numMACoeff;kQ++){
		        for (int i=kQ+1; i>=1;i--){
		            Q[i] = Q[i]-rQ[kQ]*Q[i-1];
		        }
		    }

		    for (int kP=0;kP<numMACoeff;kP++){
		        for (int i=kP+1; i>=1;i--){
		            P[i] = P[i]-rP[kP]*P[i-1];
		        }
		    }

		    float vp[2];
		    vp[0]=1;
		    vp[1]=-1;
		    int mp=numMACoeff+1;
		    int np=2;
                    //form difference filter by including root at z=1
                    //P1 = conv(P,[1 -1])
		    for (int kp=1; kp<mp+np;kp++){
		        int j1p=max(1,kp+1-np);
		        int j2p=min(kp,mp);
		        P1[kp-1]=0;
		        for (int jp=j1p; jp<=j2p;jp++){
		            P1[kp-1]=P1[kp-1]+P[jp-1]*vp[kp-jp];
		        }
		    }

		    float vq[2];
		    vq[0]=1;
		    vq[1]=1;
		    int mq=mp;
		    int nq=np;
                    //form sum filter by including root at z=-1
                    //Q1 = conv(Q,[1 1])
		    for (int kq=1; kq<mq+nq;kq++){
		        int j1q=max(1,kq+1-nq);
		        int j2q=min(kq,mq);
		        Q1[kq-1]=0;
		        for (int jq=j1q; jq<=j2q;jq++){
		            Q1[kq-1]=Q1[kq-1]+Q[jq-1]*vq[kq-jq];
		        }
		    }

		    //Average the real values for P and Q
		    for (int i=0;i<numMACoeff+1;i++){
		        MA[i]=(P1[i].real()+Q1[i].real())/2;
		    }
		}//end even number of coefficients case
		else {//odd number of coefficients... same thing, but shifted

		    for (int i=(numMACoeff+1)/2;i<numMACoeff+1;i++){//add the conjugates of the values to the end of the lists
		        rQ[i]=conj(rQ[i-(numMACoeff+1)/2]);
		    }
		    for (int i=(numMACoeff-1)/2;i<numMACoeff-1;i++){
		        rP[i]=conj(rP[i-(numMACoeff-1)/2]);
		    }
                    //P and Q are vectors of 0, starting with a 1
		    P[0]=1;
		    Q[0]=1;
		    for (int i=1;i<=numMACoeff+1;i++){
		        Q[i]=0;
		    }

		    for (int i=1;i<=numMACoeff-1;i++){
		        P[i]=0;
		    }

                    //Form the polynomials P and Q, these should be real
		    for (int kQ=0;kQ<numMACoeff+1;kQ++){
		        for (int i=kQ+1; i>=1;i--){
		            Q[i] = Q[i]-rQ[kQ]*Q[i-1];
		        }
		    }

		    for (int kP=0;kP<numMACoeff-1;kP++){
		        for (int i=kP+1; i>=1;i--){
		            P[i] = P[i]-rP[kP]*P[i-1];
		        }
		    }

		    float v[3];
		    v[0]=1;
		    v[1]=0;
		    v[2]=-1;
		    int m=numMACoeff;
		    int n=3;
                    //form difference filter by including root at z=+1 and z=-1
                    //P1 = conv(P,[1 0 -1])
		    for (int k=1; k<m+n; k++){
		        int j1= max(1,k+1-n);
		        int j2= min(k,m);
		        P1[k-1]=0;
		        for (int j=j1; j<=j2;j++){
		            P1[k-1]=P1[k-1]+P[j-1]*v[k-j];
		        }
		    }
                    //Average the real values for P and Q
		    for (int i=0;i<numMACoeff+1;i++){
		        MA[i]=(P1[i].real()+Q[i].real())/2;
		    }
		}// end of odd number of coefficients

                //Update appropriate buffer with AR coefficients
		for(int i=1; i<=numMACoeff; i++){
		    if (SynthesisFlag_Buffer1){
		        filtMACoeff_buf2[i-1]=MA[i];
		    }
		    else{
		        filtMACoeff_buf1[i-1]=MA[i];
		    }
		}
	} //end transforming MA LSFs to coefficients

    /******************************************************************************
    Interpolate variance and gain
    ******************************************************************************/
    // if buffer1 is in use then modify buffer2 (double buffering strategy)
    if(SynthesisFlag_Buffer1)
    {
        // interpolate the variance
        filtVariance_buf2 = ((hashMap[t[0]-1].variance) * BC1) + ((hashMap[t[1]-1].variance) * BC2) + ((hashMap[t[2]-1].variance) * BC3);
	if(isARMA) //interpolate the gain if ARMA
		filtGain_buf2 = ((hashMap[t[0]-1].gain) * BC1) + ((hashMap[t[1]-1].gain) * BC2) + ((hashMap[t[2]-1].gain) * BC3);
        //flip the flag now that the data is ready
        SynthesisFlag_Buffer1 = false;   
    }

    else
    {
        // interpolate the variance
        filtVariance_buf1 = ((hashMap[t[0]-1].variance) * BC1) + ((hashMap[t[1]-1].variance) * BC2) + ((hashMap[t[2]-1].variance) * BC3);
	if(isARMA) //interpolate gain if ARMA
	        filtGain_buf1 = ((hashMap[t[0]-1].gain) * BC1) + ((hashMap[t[1]-1].gain) * BC2) + ((hashMap[t[2]-1].gain) * BC3);
        //flip the flag now that the data is ready
        SynthesisFlag_Buffer1 = true;
    }
}

/******************************************************************************
  Return the force hash value BELOW our selected entry (used for placement of
  models in storage)
******************************************************************************/
int AccSynthHashTable::forceHashVal(AccSynthHashEntry hashEntry, int numMod, float speedMod[], float forceMod[])
{
	int retval = 0;

	for (int i=0;i<numMod;i++)
	{
	    if (hashEntry.force==forceMod[i] && hashEntry.speed==speedMod[i])
	    {
	        retval = i;
		break;
	    }
	}
    
    return retval;
}

/******************************************************************************
  Add entry to AccSynthHashTable
******************************************************************************/
void AccSynthHashTable::AddEntry(AccSynthHashEntry hashEntry, int numMod, float speedMod[], float forceMod[])
{
    (hashMap[forceHashVal(hashEntry, numMod, speedMod, forceMod)]) = hashEntry;
}

/******************************************************************************
  Constructor to create a blank Hash Matrix full of n HashTables for each n surface
******************************************************************************/
AccSynthHashMatrix::AccSynthHashMatrix(int numSurfaces)
{
    // store the # of surfaces in our hash table matrix
    numSurf = numSurfaces;

    // initialize an array to hold a hash table for each surface
    hashTable = new AccSynthHashTable[numSurfaces];

    fflush(stdin);
}

/******************************************************************************
  Empty constructor
******************************************************************************/
AccSynthHashMatrix::AccSynthHashMatrix::AccSynthHashMatrix()
{}

/******************************************************************************
  Add table to AccSynthHashMatrix
******************************************************************************/
void AccSynthHashMatrix::AddTable(int surfNum, int numMod, float speedMod[], float forceMod[])
{
   hashTable[surfNum] = AccSynthHashTable(surfNum,numMod,speedMod,forceMod);
   fflush(stdin);
}

/******************************************************************************
  AccSynthHashMatrix destructor
******************************************************************************/
AccSynthHashMatrix::~AccSynthHashMatrix()
{
    //delete ALL the memory from our data structure, even subclasses, so that things are deleted in the proper order
    for(int i = 0; i < numSurf; i++)
    {
        delete [] hashTable[i].hashMap;
    }

    delete [] hashTable;

}


/******************************************************************************
  Add entry to AccSynthHashMatrix
******************************************************************************/
void AccSynthHashMatrix::AddEntry(AccSynthHashEntry hashEntry, int numMod, float speedMod[], float forceMod[])
{
    hashTable[hashEntry.surfType].AddEntry(hashEntry, numMod, speedMod, forceMod);
}

/******************************************************************************
  Handles interpolate between models
******************************************************************************/
void AccSynthHashMatrix::HashAndInterp2(int interpSurf, float interpSpeed, float interpForce)
{
    hashTable[interpSurf].HashAndInterp2(interpSpeed,interpForce);
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
