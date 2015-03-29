/*
 * Copyright (C) 2015 The Paparazzi Community
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
 * @file modules/computer_vision/opticflow/opticflow_thread.c
 *
 */

// Sockets
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "opticflow_thread.h"


/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

// Video
#include "v4l/video.h"
#include "resize.h"

// Payload Code
#include "visual_estimator.h"
#include <stdio.h>
#define DEBUG_INFO(X, ...) ;


static volatile enum{RUN,EXIT} computer_vision_thread_command = RUN;  /** request to close: set to 1 */

void computervision_thread_request_exit(void) {
  computer_vision_thread_command = EXIT;
}

void *computervision_thread_main(void *args)
{
  int thread_socket = *(int *) args;

  // Local data in/out
  struct CVresults vision_results;
  struct PPRZinfo autopilot_data;

  // Status
  computer_vision_thread_command = RUN;


  /* On ARDrone2:
   * video1 = front camera; video2 = bottom camera
   */
  // Create a V4L2 device
//  struct v4l2_device *dev = v4l2_init("/dev/video2", 320, 240, 10);
  struct v4l2_device *dev = v4l2_init("/dev/video1", 1280, 720, 10);
  if (dev == NULL) {
    printf("Error initialising video\n");
    return 0;
  }

  // Start the streaming on the V4L2 device
  if(!v4l2_start_capture(dev)) {
    printf("Could not start capture\n");
    return 0;
  }

  // First Apply Settings before init
  opticflow_plugin_init(dev->w, dev->h, &vision_results);

  while (computer_vision_thread_command == RUN) {

    // Wait for a new frame
    struct v4l2_img_buf *img = v4l2_image_get(dev); 

    // Get most recent State information
    int bytes_read = sizeof(autopilot_data);
    while (bytes_read == sizeof(autopilot_data))
    {
      bytes_read = recv(thread_socket, &autopilot_data, sizeof(autopilot_data), MSG_DONTWAIT);
      if (bytes_read != sizeof(autopilot_data)) {
        if (bytes_read != -1) {
          printf("[thread] Failed to read %d bytes PPRZ info from socket.\n",bytes_read);
        }
      }
    }
    DEBUG_INFO("[thread] Read # %d\n",autopilot_data.cnt);

    // Run Image Processing with image and data and get results
    opticflow_plugin_run(img->buf, &autopilot_data, &vision_results);

    //printf("Vision result %f %f\n", vision_results.Velx, vision_results.Vely);

    /* Send results to main */
    vision_results.cnt++;
    int bytes_written = write(thread_socket, &vision_results, sizeof(vision_results));
    if (bytes_written != sizeof(vision_results)){
      perror("[thread] Failed to write to socket.\n");
    }
    DEBUG_INFO("[thread] Write # %d, (bytes %d)\n",vision_results.cnt, bytes_written);

    // Free the image
    v4l2_image_free(dev, img);
  }

  printf("Thread Closed\n");
  v4l2_close(dev);
  return 0;
}
