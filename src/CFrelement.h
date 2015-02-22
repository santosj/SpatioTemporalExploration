#ifndef CFRELEMENT_H
#define CFRELEMENT_H

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "CTimer.h"

#define MAX_ADAPTIVE_ORDER 4
#define NUM_PERIODICITIES 24 
#define FREMEN_AMPLITUDE_THRESHOLD 0.1
	
/**
@author Tom Krajnik
*/
typedef struct{
	float realStates;
	float imagStates;
	float realBalance;
	float imagBalance;
	float amplitude;
	float phase;
	float period;	
}SFrelement;

using namespace std;

class CFrelement
{
	public:
		CFrelement();
		~CFrelement();

		//adds a serie of measurements to the data
		int add(uint32_t times[],unsigned char states[],int length);

		//estimates the probability for the given times 
		int estimate(uint32_t times[],float probs[],int length,int order);

		//estimates for a single time 
		float estimate(uint32_t time,int order);

		//estimates the state's entropy for the given times 
		int estimateEntropy(uint32_t times[],float entropy[],int length,int order);

		//evaluates the error of the predictions for the given times and measurements
		int evaluate(uint32_t* times,unsigned char* signal,int length,int orderi,float* evals);
	
		void update(int modelOrder);
		void print(bool verbose=true);

		int save(FILE* file,bool lossy = false);
		int load(FILE* file);
		int save(char* name,bool lossy = false);
		int load(char* name);
		
		float gain;
		SFrelement frelements[NUM_PERIODICITIES];
		int measurements,order;
		int64_t firstTime;
		int64_t  lastTime;
};

#endif
