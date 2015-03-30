/*
 * Copyright (C) 2014 Tamas Szabo
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/cv/opticflow/harris.c
 * @brief Harris corner & edge detector
 *
 */

#include <math.h>
#include <string.h>     // mallox
#include <stdlib.h>     // realloc, free, exit, NULL
#include <stdio.h>

#include "harris.h"

#define XI(x,y,w) ((y*w)+x)
#define ABS(a) ((a)<0 ? -(a) : (a))

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////// Support Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Generate a gaussian patch of (size x size)
float* gaussianPatch(unsigned int *size, float sigma)
{
  unsigned int s = *size/2;
  *size = 2*s+1;

  float tmp[*size];
  int   cnt=0;

  for (int i=-s; i < s; ++i)
    tmp[cnt++] = i;
  
  float *out = malloc((*size)*(*size)*sizeof(float));

  sigma = 2*sigma*sigma;
  cnt=0;
  for (int i=0; i < *size; ++i)
    for (int j=0; j < *size; ++j)
      out[cnt++] = exp(-((tmp[i]*tmp[i]) + (tmp[j]*tmp[j]))/(sigma));

  return out;
}

// 2D Convolution - would be faster with fft, but what can you do ... 
void conv2(float *im, unsigned int wI, unsigned int hI, float *kernel, unsigned int wK, unsigned int hK)
{
  double tmp;

  // Half kernel sizes
  int wKh = wK/2;
  int hKh = hK/2;

  // New image buffer - will be copied into original image later
  float out[wI*hI];

  for (int i=0; i < wI; ++i)   // Image X
  {
    for (int j=0; j < hI; ++j) // Image Y
    {
      tmp = 0;          // value of the pixel - must be double as the kernel is float + it will be the sum of all pixels around the current one
      float wSum = 0;   // Weight of the kernel

      // Sum all values when the kernel is applied
      for (int ki=0; ki < wK; ++ki)
      {
        for (int kj=0; kj < hK; ++kj)
        {
          if (  ((i-wKh+ki) < 0)  ||
                ((j-hKh+kj) < 0)  ||
                ((i-wKh+ki) > wI) ||
                ((j-hKh+kj) > wI)
             ) continue;

          tmp   += kernel[XI(ki,kj,wK)]*((float)im[XI((i-wKh+ki),(j-hKh+kj),wI)]);
          wSum  += kernel[XI(ki,kj,wK)];
        }
      }
      // Assign the output's value
      out[XI(i,j,wI)] = ((unsigned char)(tmp/wSum)); 
    }
  }
  memcpy(im,&out,wI*hI);
}
xyPos* localMinima2D(float *im, unsigned int w, unsigned int h, float th, unsigned int *nPoints, unsigned char searchType)
{
  // Normalize corner metric image, so proper thresholding can be used

  if (searchType == HARRIS_LOOKFOR_EDGES)
  // Find points along edges (negative corner response)
  {
    float maxVal = 0;
    for (int i=0; i < w*h; ++i)
      if (im[i] < -maxVal)
        maxVal = -im[i];

    for (int i=0; i < w*h; ++i)
      im[i] /= maxVal;

    printf("maxVal: %.3f ", maxVal);
  }
  else if (searchType == HARRIS_LOOKFOR_CORNERS)
  // Look for corners only
  {
    float maxVal = 0;
    for (int i=0; i < w*h; ++i)
    {
      if (im[i] > maxVal)
        maxVal = -im[i];
      else if (im[i] < 0)
        im[i] = 0;
    }

    for (int i=0; i < w*h; ++i)
      im[i] = (-im[i])/maxVal;
  }
  else
  // Look for everything
  {
    float maxVal = 0;
    for (int i=0; i < w*h; ++i)
    {
      if (im[i] > maxVal)
        maxVal = im[i];
      else if (im[i] < -maxVal)
        maxVal = -im[i];
    }

    for (int i=0; i < w*h; ++i)
      im[i] = (-ABS(im[i]))/maxVal;
  }

  // Prepare output
  *nPoints    = 0;
  xyPos *out  = NULL;

  // Go through the image
  for (int i=1; i < w-1; ++i)
  {
    for (int j=1; j < h-1; ++j)
    {
      float cP = im[XI(i,j,w)];
      if (cP > -th) // If the corner metric is below the threshold, it is irrelevant
        continue;

      // Check the current point against its neighbours
      if ( (cP < im[XI(i-1,j,w)])   && (cP < im[XI(i+1,j,w)])   && (cP < im[XI(i,j-1,w)])   && (cP < im[XI(i,j+1,w)]) &&
        	 (cP < im[XI(i-1,j+1,w)]) && (cP < im[XI(i-1,j-1,w)]) && (cP < im[XI(i+1,j-1,w)]) && (cP < im[XI(i+1,j+1,w)]) 
         )
      {
        // Add point to return vector
        out = realloc(out, ((*nPoints)+1)*sizeof(xyPos));
        out[*nPoints].x = i;
        out[*nPoints].y = j;
        (*nPoints)++;
      }
    } 
  }
  return out;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////// Access functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void harris_init(harris_data *data, unsigned int patchSize, float patchSigma, float detectionThreshold, float k)
{
  data->gKernelSize   = patchSize;
  data->gKernel       = gaussianPatch((unsigned int*)&data->gKernelSize, patchSigma);

  data->edgeKernel    = malloc(3);
  data->edgeKernel[0] = -1; data->edgeKernel[1] = 0; data->edgeKernel[2] = 1;

  data->eKernelSize   = 3;

  data->mTh           = detectionThreshold;
  data->k             = k;
}

void harris_free(harris_data *data)
{
  free(data->gKernel);
  free(data->edgeKernel);
}

xyPos* harris(harris_data *data, unsigned char *image, unsigned int w, unsigned int h, unsigned int *nFeatures, unsigned char searchType)
{
  int nPix = w*h;
  float A[nPix];
  float B[nPix];
  float C[nPix];

  // Make copies of the image, so the original doesn't get destroyed
  for (int i=0; i < nPix; ++i) // Since the image in unsigned char and A, B & C are floats
    A[i] = (float)image[i];
  memcpy(&B,&A,w*h*sizeof(float));

  conv2((float*)&A, w, h, data->edgeKernel, data->eKernelSize, 1);
  conv2((float*)&B, w, h, data->edgeKernel, 1, data->eKernelSize);

  for (int i=0; i < nPix; ++i)
  {
    C[i] = A[i]*B[i];
    A[i] = A[i]*A[i];
    B[i] = B[i]*B[i];
  }
  conv2((float*)&A, w, h, data->gKernel, data->gKernelSize, data->gKernelSize);
  conv2((float*)&B, w, h, data->gKernel, data->gKernelSize, data->gKernelSize);
  conv2((float*)&C, w, h, data->gKernel, data->gKernelSize, data->gKernelSize);
  // Calculate the corner metric
  for (int i=0; i < nPix; ++i)
    A[i] = A[i]*B[i] - C[i]*C[i] - data->k*(A[i]+B[i]);

  *nFeatures = 0;

  // Find the local extremes in the corner metric
  xyPos* out = localMinima2D((float*)&A, w, h, data->mTh, nFeatures, searchType);

  return out;
}
