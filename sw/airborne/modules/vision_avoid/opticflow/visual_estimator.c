/*
 * Copyright (C) 2014 Hann Woei Ho
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
 * @file modules/computer_vision/opticflow/visual_estimator.c
 * @brief Estimate velocity from optic flow.
 *
 * Using sensors from vertical camera and IMU of Parrot AR.Drone 2.0.
 *
 * Warning: all this code is called form the Vision-Thread: do not access any autopilot data in here.
 */

#include "std.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Own Header
#include "visual_estimator.h"

// Computer Vision
#include "opticflow/optic_flow_int.h"
#include "opticflow/fast9/fastRosten.h"

// for FPS
#include "modules/computer_vision/cv/framerate.h"

// Avoidance headers
#include "../avoid_nav_transportFcns.h"

// PeakFinder function
#include "../peakfinder.h"
#include "../avoid_nav.h"

#include "encoding/rtp.h"

#include "resize.h"
#include "image.h"

// Downlink Video
#define DOWNLINK_VIDEO 	          
#define DEBUG_VIDEO 	            1
#define DEBUG_MARK_FEATUREPOINTS  1
#define DEBUG_MARK_FLOWSUM        1

#define DEBUG_OVERLAY_COLOR       255

#define DOWNLINK_FLOWSUM 1

#ifdef DOWNLINK_FLOWSUM
   struct UdpSocket *flowSock;
#endif

#ifdef DOWNLINK_VIDEO
   #include "encoding/jpeg.h"
   struct UdpSocket *vsock;

   uint8_t *downlinkBuffer;
   uint8_t *downlinkBufferGray;
#endif

// This will downscale the front camera image from (1280x720) to (320x180)
//#define IMAGE_DOWNSIZE_FACTOR 4
#define IMAGE_DOWNSIZE_FACTOR 4



uint8_t *jpegbuf;

// Local variables
struct visual_estimator_struct
{
  // Image size
  unsigned int imgWidth;
  unsigned int imgHeight;

  unsigned int inFrameWidth;
  unsigned int inFrameHeight;

  // Images
//  struct img_struct input_frame;
//  struct img_struct current_frame;
  uint8_t *current_frame;
  uint8_t *prev_frame;

  uint8_t *gray_frame;
  uint8_t *prev_gray_frame;

  // Initialization
  int old_img_init;

  // Store previous
  float prev_pitch;
  float prev_roll;
} visual_estimator;

// ARDrone Vertical Camera Parameters
#define FOV_H 0.67020643276
#define FOV_W 0.89360857702
#define Fx_ARdrone 343.1211
#define Fy_ARdrone 348.5053

// Corner Detection
#define MAX_FEATURE_COUNT 100
#define MAX_FLOW_COUNT    25

// Flow Derotation
#define FLOW_DEROTATION

// Peakdetector Threshold
#define PEAKDETECTOR_THRESHOLD 0.5

// Called by plugin
void opticflow_plugin_init(unsigned int w, unsigned int h, struct CVresults *results)
{
  // Initialize variables
  visual_estimator.inFrameWidth   = w;
  visual_estimator.inFrameHeight  = h;

// Only the incoming frame has the complete size.
  w = w / IMAGE_DOWNSIZE_FACTOR;
  h = h / IMAGE_DOWNSIZE_FACTOR;

  visual_estimator.current_frame    = (uint8_t *)malloc(w * h * 2);
  visual_estimator.prev_frame       = (uint8_t *)malloc(w * h * 2);
  visual_estimator.gray_frame       = (unsigned char *) calloc(w * h, sizeof(uint8_t));
  visual_estimator.prev_gray_frame  = (unsigned char *) calloc(w * h, sizeof(uint8_t));

  visual_estimator.old_img_init = 1;
  visual_estimator.prev_pitch = 0.0;
  visual_estimator.prev_roll = 0.0;

  visual_estimator.imgWidth = w;
  visual_estimator.imgHeight = h;

  results->cnt          = 0;
  results->flow_count   = 0;
  results->count        = 0;
  results->FPS          = 0;
  
  results->WP_pos_X     = 0;
  results->WP_pos_Y     = 0;
  results->head_cmd     = 0;
  
  framerate_init();

// Initialize obstacle map  
  init_map();
  vehicle_cache_init();


#ifdef DOWNLINK_VIDEO
  // Video Compression
  downlinkBufferGray = (uint8_t *)malloc(w * h);
  downlinkBuffer     = (uint8_t *)malloc(w * h * 2);
  jpegbuf            = (uint8_t *)malloc(w * h * 2);

  // Network Transmit
  vsock = udp_socket("192.168.1.255", 5000, 5001, FMS_BROADCAST);
#endif
#ifdef DOWNLINK_FLOWSUM
  flowSock = udp_socket("192.168.1.255", 9000, 9001, FMS_BROADCAST);;
#endif
}

void opticflow_plugin_run(unsigned char *frame, struct PPRZinfo* info, struct CVresults *results)
{
  // Corner Tracking
  // Working Variables
//  int max_count = 25;
  int borderx = 24, bordery = 24;
  int x[MAX_FEATURE_COUNT], y[MAX_FEATURE_COUNT];
  int new_x[MAX_FEATURE_COUNT], new_y[MAX_FEATURE_COUNT];
  int status[MAX_FEATURE_COUNT];
  int dx[MAX_FEATURE_COUNT], dy[MAX_FEATURE_COUNT];
  int w = visual_estimator.imgWidth;
  int h = visual_estimator.imgHeight;


  // Framerate Measuring
  results->FPS = framerate_run();

  printf("visual_estimator.c: Current FPS: %.2f ",results->FPS);

  // Downsize the image for processing
  ImResizeUYVU(visual_estimator.current_frame, visual_estimator.imgWidth, visual_estimator.imgHeight, 
                frame, visual_estimator.inFrameWidth, visual_estimator.inFrameHeight, 
              IMAGE_DOWNSIZE_FACTOR);

  // Get Grayscale image
  CvtYUYV2Gray(visual_estimator.gray_frame, visual_estimator.current_frame, w, h);


  // Initialize prev_frame & prev_gray_frame to the current frames
  if (visual_estimator.old_img_init == 1) {
    memcpy(visual_estimator.prev_frame, frame, w * h * 2);
    memcpy(visual_estimator.prev_gray_frame, visual_estimator.gray_frame, w * h);
    visual_estimator.old_img_init = 0;
  }

#ifdef DOWNLINK_VIDEO
 // Make a copy of the current frame, so we can add stuff to it for debuging
 memcpy(downlinkBufferGray, visual_estimator.prev_gray_frame, w*h);
#endif

  // *************************************************************************************
  // Corner detection
  // *************************************************************************************

  // FAST corner detection
  int fast_threshold = 20;
  xyFAST *pnts_fast;
  pnts_fast = fast9_detect((const byte *)visual_estimator.prev_gray_frame, w, h, w,
                           fast_threshold, &results->count);

  printf("fP: %d ", results->count);
  // Remove neighboring corners
  int min_distance = 5;
  int min_distance2 = min_distance * min_distance;

  unsigned char labelRemove[results->count];
  //Ensure that the labelRemove values are initialized to zero
  for (int i = 0; i < results->count; i++)
    labelRemove[i] = 0;

  for (int i = 0; i < results->count; i++) 
  {
    if (labelRemove[i]) //The point is already marked to be removed *flies away*
        continue;

    for (int j = i+1; j < results->count; j++) 
    {
      if (labelRemove[j]) //The point is already marked to be removed *flies away*
        continue;

      // distance squared:
      int p_dx = pnts_fast[i].x - pnts_fast[j].x;
      int p_dy = pnts_fast[i].y - pnts_fast[j].y;
//      float distance2 = (x[i] - x[j]) * (x[i] - x[j]) + (y[i] - y[j]) * (y[i] - y[j]);
      int distance2 = p_dx*p_dx + p_dy*p_dy;;
      if (distance2 < min_distance2)
        labelRemove[j] = 1;
    }
  }

  // Get the first at most MAX_FEATURE_COUNT sparsely spead points (as per labelRemove!)
  int srcPos  = 0;
  int destPos = 0;
  while ( (srcPos < results->count) && (destPos < MAX_FEATURE_COUNT) )
  {
    if (!labelRemove[srcPos])
    {
      x[destPos] = pnts_fast[srcPos].x;
      y[destPos] = pnts_fast[srcPos].y;

#ifdef DOWNLINK_VIDEO
  #ifdef DEBUG_VIDEO
    #ifdef DEBUG_MARK_FEATUREPOINTS

 // Mark the feature points on the downlinked image as white (255)! (note: 2x2 pixels should be marked, otherwise they disappear after jpeg compression)
  downlinkBufferGray[w*y[destPos] + x[destPos]]   = DEBUG_OVERLAY_COLOR;
  downlinkBufferGray[w*y[destPos] + x[destPos]+1] = DEBUG_OVERLAY_COLOR;

  if (x[destPos] < (w-2) )
  {
    downlinkBufferGray[w*y[destPos] + x[destPos] + w]    = DEBUG_OVERLAY_COLOR;
    downlinkBufferGray[w*y[destPos] + x[destPos] + w +1] = DEBUG_OVERLAY_COLOR;
  }
  else
  {
    downlinkBufferGray[w*y[destPos] + x[destPos] + w]    = DEBUG_OVERLAY_COLOR;
    downlinkBufferGray[w*y[destPos] + x[destPos] + w +1] = DEBUG_OVERLAY_COLOR;
  }
    #endif
  #endif
#endif

      ++destPos;
    }
    ++srcPos;
  }
  results->count = destPos;

  free(pnts_fast);


//  #warning !!!!!!!!!!!!!!!!! CORNER TRACKING DISABLED !!!!!!!!!!!!!!!!!!!

  // *************************************************************************************
  // Corner Tracking
  // *************************************************************************************
//  opticFlowLK(visual_estimator.gray_frame, visual_estimator.prev_gray_frame, x, y,
//              count_fil, w, h, new_x, new_y, status, 5, 100);

  results->flow_count = results->count;

  opticFlowLK(visual_estimator.gray_frame, visual_estimator.prev_gray_frame, x, y,
              results->flow_count, w, h, new_x, new_y, status, 5, 100);

  // Remove the invalid points
  srcPos  = 0;
  destPos = 0;
  for (srcPos = 0; srcPos < results->flow_count; ++srcPos)
  {
    if (status[srcPos] && !(new_x[srcPos] < borderx || new_x[srcPos] > (w - 1 - borderx) ||
                 new_y[srcPos] < bordery || new_y[srcPos] > (h - 1 - bordery)))
    {
      if (srcPos != destPos) // No point in copying to the same place... is there?
      {
        x[destPos]      = x[srcPos];
        y[destPos]      = y[srcPos];
        new_x[destPos]  = new_x[srcPos];
        new_y[destPos]  = new_y[srcPos];
      }
      ++destPos;
    }
  }
  results->flow_count = destPos;

/*  for (int i = results->flow_count - 1; i >= 0; i--) 
  {
    int remove_point = 1;

    if (status[i] && !(new_x[i] < borderx || new_x[i] > (w - 1 - borderx) ||
                       new_y[i] < bordery || new_y[i] > (h - 1 - bordery))) 
    {
      remove_point = 0;
    }

    if (remove_point) 
    {
      for (int c = i; c < results->flow_count - 1; c++) 
      {
        x[c] = x[c + 1];
        y[c] = y[c + 1];
        new_x[c] = new_x[c + 1];
        new_y[c] = new_y[c + 1];
      }
      results->flow_count--;
    }
  }
*/
  // Optical Flow Computation
  for (int i = 0; i < results->flow_count; i++) 
  {
    dx[i] = new_x[i] - x[i];
    dy[i] = new_y[i] - y[i];
  }

/*  results->dx_sum = 0.0;
  results->dy_sum = 0.0;
  // Median Filter
  if (results->flow_count) {
    quick_sort_int(dx, results->flow_count); // 11
    quick_sort_int(dy, results->flow_count); // 11

    results->dx_sum = (float) dx[results->flow_count / 2];
    results->dy_sum = (float) dy[results->flow_count / 2];
  } else {
    results->dx_sum = 0.0;
    results->dy_sum = 0.0;
  }*/

  // Flow Derotation
  float diff_pitch = (info->theta - visual_estimator.prev_pitch) * h / FOV_H;
  float diff_roll = (info->phi - visual_estimator.prev_roll) * w / FOV_W;
  visual_estimator.prev_pitch = info->theta;
  visual_estimator.prev_roll = info->phi;

//  float OFx_trans, OFy_trans;
#ifdef FLOW_DEROTATION/*
  if (results->flow_count) {
    OFx_trans = results->dx_sum - diff_roll;
    OFy_trans = results->dy_sum - diff_pitch;

    if ((OFx_trans <= 0) != (results->dx_sum <= 0)) {
      OFx_trans = 0;
      OFy_trans = 0;
    }
  } else {
    OFx_trans = results->dx_sum;
    OFy_trans = results->dy_sum;
  }
#else
  OFx_trans = results->dx_sum;
  OFy_trans = results->dy_sum;*/
#endif


  // *************************************************************************************
  // This is where the avoidance magic happen!
  // *************************************************************************************
  
  navTransportData.stateEnuPosX     = info->enuPosX;
  navTransportData.stateEnuPosY     = info->enuPosY;
  navTransportData.stateEnuHeading  = info->psi;
  
  navTransportData.stateWpStatus    = (info->targetDist < 0.1f); // Threshold on reaching the target is set to 0.1 meters!
  
  float peakAngles[10];
  int   numPeaks = 10;

//  peakfinder (int Ncols, int Nflows, int *hPos, int *hFlow, float threshold, float vangle, int *np, float * angle);
//  peakfinder(w, results->flow_count, (int*)&new_x, (int*)&dx, PEAKDETECTOR_THRESHOLD, FOV_W, &numPeaks, (float*)&peakAngles);
  
  float flowSum[w];
  float maxFlow;

  printf("maxFlowSum: %.2f nF: %d nP: %d ", maxFlow, results->flow_count, results->count);
  cv_flowSum((int*)&x, (int*)&dx, results->flow_count, w, (float*)&flowSum, &maxFlow);
  if (maxFlow < 0.001)
    maxFlow = 0.001;
  cv_smoothAndNormalizeSum(flowSum, w, maxFlow, 20);

#ifdef DOWNLINK_FLOWSUM
  {
    unsigned char numSeg = 20;
    unsigned char segVals[numSeg];

    int numPixPerSeg = (w/numSeg);
    for (int seg=0; seg < numSeg; ++seg)
    {
      float segSum = 0;
      for (int px = seg*numPixPerSeg; px < (seg+1)*numPixPerSeg; ++px)
        segSum += (flowSum[px]/maxFlow);
      
      segVals[seg] = (unsigned char)segSum;
    }
    
    if (udp_write(flowSock, (unsigned char*)&segVals, numSeg) != numSeg)
      printf("UDP write error! ");
  }
#endif

#ifdef DOWNLINK_VIDEO
  #ifdef DEBUG_VIDEO
    #ifdef DEBUG_MARK_FLOWSUM
  for (int i=0; i < w; ++i)
  {
    int mH = (flowSum[w]/maxFlow)*(h-2)+1;
    downlinkBufferGray[i + mH*w]      = DEBUG_OVERLAY_COLOR;
    downlinkBufferGray[i + (mH-1)*w]  = DEBUG_OVERLAY_COLOR;
    if (i == 0)
    {
      downlinkBufferGray[i + mH*w + 1]      = DEBUG_OVERLAY_COLOR;
      downlinkBufferGray[i + (mH-1)*w + 1]  = DEBUG_OVERLAY_COLOR;
    }
    else
    {
      downlinkBufferGray[i + mH*w - 1]      = DEBUG_OVERLAY_COLOR;
      downlinkBufferGray[i + (mH-1)*w - 1]  = DEBUG_OVERLAY_COLOR;
    }
  }
    #endif
  #endif
#endif

  #warning !!!!!!!!!!!!!!!!!Navigation Disabled in visual_estimator.c!!!!!!!!!!!!!!!!!!!
  cv_peakFinder((float*)&flowSum, w, PEAKDETECTOR_THRESHOLD, &numPeaks, (float*)&peakAngles, FOV_W);
//  navigate();

  results->WP_pos_X     = navTransportData.currentWpLocationX;
  results->WP_pos_Y     = navTransportData.currentWpLocationY;
  results->head_cmd     = navTransportData.currentHeadingSetpoint;

#ifdef DOWNLINK_VIDEO
    // JPEG encode the image:
//    uint32_t quality_factor = 5; //20 if no resize,
    uint32_t quality_factor = 20; //20 if no resize,
    uint8_t dri_header = 0;
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)

    // Convert back the grayscale image to yuyv as the jpeg encoder expects this format! The color codes are all set to zero (grayscale)
    ImGray2UYVU(downlinkBuffer, downlinkBufferGray, w, h);

    // Send the small image, not the full-sized one!
    uint8_t *end = encode_image(downlinkBuffer, jpegbuf, quality_factor, image_format, w, h, dri_header);
    
    uint32_t size = end - (jpegbuf);

    //printf("Sending an image ...%u\n", size);
    send_rtp_frame(vsock, jpegbuf, size, w, h, 0, quality_factor, dri_header, 0);
#endif


  // *************************************************************************************
  // Next Loop Preparation
  // *************************************************************************************

  memcpy(visual_estimator.prev_frame, frame, w * h * 2);
  memcpy(visual_estimator.prev_gray_frame, visual_estimator.gray_frame, w * h);
  printf("\n");
}
void ImGray2UYVU(unsigned char *frame, unsigned char *grayFrame, int imW, int imH)
{
  ++grayFrame;
  for (int i=0; i < imW*imH; ++i)
  {
    *frame++ = 127; 		      // U
    *frame++ = *grayFrame++; 	// Y
    ++grayFrame; 		          // <= NO idea why this is necessary!
  }
}
void ImUYVU2Gray(unsigned char *grayFrame, unsigned char *frame, int imW, int imH)
{
  for (int i=0; i < imW*imH; ++i)
  {
    *grayFrame++ = *frame++;
    ++frame;
  }
}
void ImResizeUYVU(unsigned char *output, int imWOut, int imHOut, unsigned char *input, int imWIn, int imHIn, int downsample)
{ // Adapted from resize.h for using the raw image buffers
  uint8_t *source = input;
  uint8_t *dest = output;

  int pixelskip = (downsample - 1) * 2;
  for (int y = 0; y < imHOut; y++) {
    for (int x = 0; x < imWOut; x += 2) {
      // YUYV
      *dest++ = *source++; // U
      *dest++ = *source++; // Y
      *dest++ = *source++; // V
      source += pixelskip;
      *dest++ = *source++; // Y
      source += pixelskip;
    }
    // read 1 in every 'downsample' rows, so skip (downsample-1) rows after reading the first
    source += (downsample-1) * imWIn * 2;
  }
}
