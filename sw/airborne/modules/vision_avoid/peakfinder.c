#include "peakfinder.h"

/*void valueIndexSort(struct valueIndexPair* data, int N)
{
  int i, j;
  struct valueIndexPair v, t;

  if(N<=1) return;

  // Partition elements
  v.value = data[0].value;
  v.index = data[0].index;
  
  i = 0;
  j = N;
  for(;;)
  {
    while(i < N && data[++i].value < v.value) { }
    while(data[--j].value > v.value) { }
    
    if(i >= j) break;
    
    t.value = data[i].value; data[i].value = data[j].value; data[j].value = t.value;
    t.index = data[i].index; data[i].index = data[j].index; data[j].index = t.index;
  }
  t.value = data[i-1].value; data[i-1].value = data[0].value; data[0].value = t.value;
  t.index = data[i-1].index; data[i-1].index = data[0].index; data[0].index = t.index;
  
  valueIndexSort(data, i-1);
  valueIndexSort(data+i, N-i);
}*/


/* Peak finder Function*/
// Note that the optical flow values is an INT, because it corresponds to the number of shifted PIXELS
void peakfinder (int Ncols, int Nflows, int *hPos, int *hFlow, float threshold, float vangle, int *np, float * angle)
{
/*
  // Code by David: assumes flow to be in matrix format, which is not the case
   
	// Inputs: horizontal optical flow matrix, and their dimensions, a threshold, the angle of vision and the pointers for the output
	// Outputs: number of peaks and a vector with the angle for where are those peaks
	
	int j, i, k;
	float val[cols], aux, aux2, w[cols+2], left[10], center[10], right[10];
	aux2 = 0;

	for(j=0; j<cols; j++)
	{
		aux = 0;	
		for(i=0; i<rows; i++)
		{
			//Makes every value of the horizontal optical flow matrix non-negative (absolute value)
			if (hflow[i*cols+j] < 0)
				hflow[i*cols+j] = hflow[i*cols+j]*(-1);  
			
			//Sum of every value on the same collumn of the horizontal optical flow matrix
			aux = aux + hflow[i*cols+j];
		}
		//New vector with the sum of each on collumn on each entry
		val[j] = aux;
		
		// Maximum value of any entry of the new vector
		if (val[j] > aux2)
			aux2 = val[j];
	}*/
	
	
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
/*	
  {  	
		flowVSum[j] = flowVSum[j]/flowMaxVal;
		
		//Values greater than the threshold will be 1 (binary)
		if (flowVSum[j] >= threshold)
			flowVSum[j] = 1;
		else
			flowVSum[j] = 0;
		
		w[j+1] = flowVSum[j];
	}
*/
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


