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
 * @file modules/computer_vision/cv/opticflow/harris.h
 * @brief Harris corner & edge detector
 *
 */
#ifndef HARRIS_H
#define HARRIS_H

#include <inttypes.h>

#define HARRIS_LOOKFOR_CORNERS  1
#define HARRIS_LOOKFOR_EDGES    2
#define HARRIS_LOOKFOR_ALL      3

typedef struct 
{ 
int x;
int y; 
} xyPos;


typedef struct
{
  float *gKernel;           // Gaussian kernel for bluring
  unsigned int gKernelSize; // Size of the gaussian kernel

  float *edgeKernel;        // Kernel used for the edge detection
  unsigned int eKernelSize; // Size of the edge kernel

  float mTh;                // Feature detection threshold
  float k;

} harris_data;


extern float* gaussianPatch(unsigned int *size, float sigma);
extern void   conv2(float *im, unsigned int wI, unsigned int hI, float *kernel, unsigned int wK, unsigned int hK);
extern xyPos* localMinima2D(float *im, unsigned int w, unsigned int h, float th, unsigned int *nPoints, unsigned char searchType);

extern void harris_init(harris_data *data, unsigned int patchSize, float patchSigma, float detectionThreshold, float k);
extern void harris_free(harris_data *data);

extern xyPos* harris(harris_data *data, unsigned char *image, unsigned int w, unsigned int h, unsigned int *nFeatures, unsigned char searchType);

#endif
