#ifndef CFREMENGRID_H
#define CFREMENGRID_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "CTimer.h" 
#include "CFrelement.h" 

/**
@author Tom Krajnik
*/

using namespace std;

class CFremenGrid
{
	public:
		CFremenGrid(float originX,float originY,float originZ,int dimX,int dimY,int dimZ,float cellSize);
		~CFremenGrid();

		/*state estimation: estimates the grid for the given time*/
		bool recalculate(uint32_t timeStamp);
		float getObtainedInformation();

		float estimate(float x,float y,float z,uint32_t timeStamp);
		float estimateInformation(float x,float y,float z,float range,uint32_t t);
		float getClosestObstacle(float x,float y,float zlimit,float range);

		/*changes the model order*/
		void print(bool verbose);
		void save(const char*name,bool lossy = false,int forceOrder = -1);
		bool load(const char*name);

		float estimate(unsigned int index,uint32_t timeStamp);
		void incorporate(float *x,float *y,float *z,float *d,int size,uint32_t t);
		int getIndex(float x,float y,float z);

		//center of the first cell
		float oX,oY,oZ;		
		//size of the grid cells
		float resolution;
		//grid dimensions
		int xDim,yDim,zDim;
		float* probs;	
		char *aux;
		int numCells;
		bool debug;

		float minProb,maxProb,residualEntropy,residualInformation;
		float lastPhiRange,lastPsiMin,lastPsiMax,lastRange;
		int *raycasters;
		int numRaycasters;
		float obtainedInformation;
		CFrelement* frelements;
};

#endif
