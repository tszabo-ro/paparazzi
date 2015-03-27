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
 * @file modules/vision_avoid/avoid_nav_transportFcns.c
 * @brief 
 */

#include "avoid_nav_transportFcns.h"

//  float stateEnuPosX;
//  float stateEnuPosY;
//  float stateEnuHeading;
//  
//  float currentWpLocationX;
//  float currentWpLocationY;
//  float currentHeadingSetpoint;
//  
//  uint8_t stateWpStatus;
  
struct navTransport navTransportData;

void     getArenaLimits(float *coords, int N)
{
  if(N > arenaLimits.arenaLimIndex)
    N = arenaLimits.arenaLimIndex;


  int i;
  for (i=0; i < N; ++i)
//  { limX[i] = arenaLimits.X[i]; limY[i] = arenaLimits.Y[i]; }
  {
    coords[i*2+0]=arenaLimits.X[i];
    coords[i*2+1]=arenaLimits.Y[i];
  }
}
void     setNewWaypointLocation(float posX, float posY, float heading)
{
  navTransportData.currentWpLocationX     = posX;
  navTransportData.currentWpLocationY     = posY;
  navTransportData.currentHeadingSetpoint = heading;
}
void     getCurrentPos(float *X, float *Y, float *heading) 
{
  *X       = navTransportData.stateEnuPosX;
  *Y       = navTransportData.stateEnuPosY;
  *heading = navTransportData.stateEnuHeading;
}
uint8_t  wpReached(void) 
{ 
  return navTransportData.stateWpStatus;
}
