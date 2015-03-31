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
 * @file modules/computer_vision/opticflow/inter_thread_data.h
 * @brief Inter-thread data structures.
 *
 * Data structures used to for inter-thread communication via Unix Domain sockets.
 */


#ifndef _INTER_THREAD_DATA_H
#define _INTER_THREAD_DATA_H

/// Data from thread to module
struct CVresults {
  unsigned int cnt;          // Number of processed frames
  unsigned int flow_count;
  unsigned int count;
  float FPS;
  
  float WP_pos_X;
  float WP_pos_Y;
  float head_cmd;
  
/*  float Velx;       // Velocity as measured by camera
  float Vely;

  float cam_h;      // Debug parameters
  float OFx, OFy, dx_sum, dy_sum;
  float diff_roll;
  float diff_pitch;*/
};

/// Data from module to thread
struct PPRZinfo {
  float phi;      // roll [rad]
  float theta;    // pitch [rad]
  float psi;      // yaw/heading [rad]
  
  float enuPosX;  // X Position in the ENU frame
  float enuPosY;  // Y Position in the ENU frame
  
  float targetDist;
};

#endif
