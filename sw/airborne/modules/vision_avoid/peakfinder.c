#include "peakfinder.h"

/*
// Peak finder Function
// Note that the optical flow values is an INT, because it corresponds to the number of shifted PIXELS
void peakfinder (int Ncols, int Nflows, int *hPos, int *hFlow, float threshold, float vangle, int *np, float * angle)
{

//  // Code by David: assumes flow to be in matrix format, which is not the case
//   
//	// Inputs: horizontal optical flow matrix, and their dimensions, a threshold, the angle of vision and the pointers for the output
//	// Outputs: number of peaks and a vector with the angle for where are those peaks
//	
//	int j, i, k;
//	float val[cols], aux, aux2, w[cols+2], left[10], center[10], right[10];
//	aux2 = 0;
//
//	for(j=0; j<cols; j++)
//	{
//		aux = 0;	
//		for(i=0; i<rows; i++)
//		{
//			//Makes every value of the horizontal optical flow matrix non-negative (absolute value)
//			if (hflow[i*cols+j] < 0)
//				hflow[i*cols+j] = hflow[i*cols+j]*(-1);  
//			
//			//Sum of every value on the same collumn of the horizontal optical flow matrix
//			aux = aux + hflow[i*cols+j];
//		}
//		//New vector with the sum of each on collumn on each entry
//		val[j] = aux;
//		
//		// Maximum value of any entry of the new vector
//		if (val[j] > aux2)
//			aux2 = val[j];
//	}
	
	
// Flow summing code by Tamas with the new data format
	float flowVSum[Ncols];
	float flowMaxVal = 0; // The minimum of the flow sum is 0
	int i;
	
	// Ensure that the flow sum is initially zero at all points
	for (i=0; i < Ncols; ++i)
  	flowVSum[i] = 0;
  
  // Add the flow data where exists
  for (i=0; i < Nflows; ++i)
    flowVSum[hPos[i]] += hFlow[i];
	
	// Find Max flow value
	for (i=0; i < Ncols; ++i)
	  if (flowMaxVal < flowVSum[i])
	    flowMaxVal = flowVSum[i];
	
	float left[10], center[10], right[10];
	float w[Ncols+2];
	
	int j,k;
	
// Code by David
	w[0] = 0; // Forcing zero on the begining and end of the new vector to force atleast one peak 
	
	for(j=0; j<Ncols; j++)
  	w[j+1] = ( (flowVSum[j]/flowMaxVal) > threshold ); // <= This is faster & cleaner; Tamas
	
//  {  	
//		flowVSum[j] = flowVSum[j]/flowMaxVal;
//		
//		//Values greater than the threshold will be 1 (binary)
//		if (flowVSum[j] >= threshold)
//			flowVSum[j] = 1;
//		else
//			flowVSum[j] = 0;
//		
//		w[j+1] = flowVSum[j];
//	}

	w[j+1] = 0;


	i = 0;
	k = 0;
	for(j=0; j<Ncols+2; ++j)// ++j is faster than j++; Tamas
	{
//	if (w[j+1]-w[j]>0.5 && i<10) // By David
    if ( (w[j+1] > w[j]) && (i<10) ) // Instead of difference & comparison, use only comparison. Since w[] is biary (1 or 0); 1-0 = 1; 0-1 = -1; you can use 0 as the threshold, in which case these comparisons work just fine; Tamas
		{
			left[i]=j; // vector with the index of the "ascending" part of the peak
			i++;
		}
//	if (w[j+1]-w[j]<-0.5 && k<10) // By David
		if ( (w[j+1] < w[j]) && (k<10) ) // Same goes here; Tamas
		{
			right[k]=j; // vector with the index of the "descending" part of the peak
			k++;
		}
	}
	*np = i; // Number of peaks
	if (i!=10)
	{
		//If there is less than 10 peaks, the other part of the vectors will be zero
		for(j=i; j<10; ++j) // ++j is faster; Tamas
		{
			right[j]=0;  
			left[j]=0;
		}

	}
	for(j=0; j<10; ++j) // ++j is faster than j++; Tamas
	{
		center[j]=(right[j]+left[j])/2; 
		angle[j] = ((center[j]/(Ncols+2))-0.5)*vangle; // Conversion of the indices to real angles
	}
}
*/
void cv_flowSum(int *hPos, int *hFlow, int NFlows, int NCols, float *flowSum, float *maxFlow)
{
// Flow summing code by Tamas with the new data format
	*maxFlow = 0; // The minimum of the flow sum is 0
	int i;
	
	// Ensure that the flow sum is initially zero at all points
	for (i=0; i < NCols; ++i)
  	flowSum[i] = 0;
  
  // Add the flow data where exists
  for (i=0; i < NFlows; ++i)
    flowSum[hPos[i]] += hFlow[i];
	
	// Find Max flow value
	for (i=0; i < NCols; ++i)
	  if (*maxFlow < flowSum[i])
	    *maxFlow = flowSum[i];

}
void cv_smoothAndNormalizeSum(float *flowSum, int NCols, float maxFlow, unsigned char smootherSize)
{

  unsigned char halfSize = smootherSize/2;
  float scale = maxFlow*smootherSize;  

  float out[smootherSize];

  for (int i=0; i < NCols; ++i)
  {
    out[i] = 0;
    for (int j=(i-halfSize); j < (i+halfSize); ++j)
    {
      if ( (j >= 0) && (j < NCols) ) // This lowers the values close to the edges of the frame
        out[i] += flowSum[j];
    }
    out[i] /= scale;
  }

  memcpy(flowSum, &out, NCols*sizeof(float));
}
void cv_peakFinder(float *flowSum, int NCols, float threshold, int *np, float *angle, float visualAngle)
{
	float left[10], center[10], right[10];
	float w[NCols+2];
	
	int i,j,k;
	w[0] = 0; // Forcing zero on the begining and end of the new vector to force atleast one peak 
	
	for(j=0; j<NCols; j++)
  	w[j+1] = ( flowSum[j] > threshold );

	w[j+1] = 0;


	i = 0;
	k = 0;
	for(j=0; j<NCols+2; ++j)// ++j is faster than j++; Tamas
	{
    if ( (w[j+1] > w[j]) && (i<10) )
		{
			left[i]=j; // vector with the index of the "ascending" part of the peak
			i++;
		}
		if ( (w[j+1] < w[j]) && (k<10) )
		{
			right[k]=j; // vector with the index of the "descending" part of the peak
			k++;
		}
	}
	*np = i; // Number of peaks
	if (i!=10)
	{
		//If there is less than 10 peaks, the other part of the vectors will be zero
		for(j=i; j<10; ++j)
		{
			right[j]=0;  
			left[j]=0;
		}

	}
	for(j=0; j<10; ++j)
	{
		center[j]=(right[j]+left[j])/2; 
		angle[j] = ((center[j]/(NCols+2))-0.5)*visualAngle; // Conversion of the indices to real angles
	}

}

