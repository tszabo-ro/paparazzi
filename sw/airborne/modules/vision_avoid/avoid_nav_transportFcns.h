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
 * @file modules/vision_avoid/avoid_nav_transportFcns.h
 * @brief 
 */

#ifndef AVOID_NAV_TRANSPORTFCNS
#define AVOID_NAV_TRANSPORTFCNS

#include "inttypes.h"

struct navTransport
{
  float stateEnuPosX;
  float stateEnuPosY;
  float stateEnuHeading;
  
  float currentWpLocationX;
  float currentWpLocationY;
  float currentHeadingSetpoint;
  
  uint8_t stateWpStatus;
};
struct arenaLimsENU
{
  float X[4];
  float Y[4];
  int arenaLimIndex;
};
struct arenaLimsENU arenaLimits;

struct flowPeaksStruct
{
  float *angles;
  int nAngles;
} flowPeaks;

extern struct navTransport navTransportData;

extern void     getArenaLimits(float *coords, int N);
extern void     setNewWaypointLocation(float posX, float posY, float heading);
extern void     getCurrentPos(float *X, float *Y, float *heading);
extern uint8_t  wpReached(void);

#endif /* OPTICFLOW_MODULE_H */
