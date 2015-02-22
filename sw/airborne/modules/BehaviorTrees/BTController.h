/*
 * Copyright (C) T. Szabo
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/BTController/BTController.h"
 * @author T. Szabo
 * Collision avoidance controller based on the Bluegiga USB dongle, RSSI2dist and BehaviorTrees
 */
#ifndef BTCONTROLLER_H
#define BTCONTROLLER_H

#define     DRONE_MAX_VCMD          0.3f
#define     DRONE_MAX_PSIDOT        (20*(M_PI/180))

#define     BTCONTROLLER_DT         0.05f
#define     GEOMETRY_SMALL_NUMBER   0.00000001f

#define     WALL_THREASHOLD         <

#define     AREALIM_P0_X           -5.269f
#define     AREALIM_P0_Y            0.379f

#define     AREALIM_P1_X           -0.438f
#define     AREALIM_P1_Y           -2.469f

#define     AREALIM_P2_X            2.578f
#define     AREALIM_P2_Y            2.449f

#define     AREALIM_P3_X           -2.227f
#define     AREALIM_P3_Y            5.289f

#define     AL_N0_Y                 (AREALIM_P0_X-AREALIM_P1_X)
#define     AL_N0_X                 (AREALIM_P1_Y-AREALIM_P0_Y)

#define     AL_N1_Y                 (AREALIM_P1_X-AREALIM_P2_X)
#define     AL_N1_X                 (AREALIM_P2_Y-AREALIM_P1_Y)

#define     AL_N2_Y                 (AREALIM_P2_X-AREALIM_P3_X)
#define     AL_N2_X                 (AREALIM_P3_Y-AREALIM_P2_Y)

#define     AL_N3_Y                 (AREALIM_P3_X-AREALIM_P0_X)
#define     AL_N3_X                 (AREALIM_P0_Y-AREALIM_P3_Y)


float psiCmd;
float psiDotCmd;
float speedCmd;

float dummy1;
float dummy2;
float dummy3;

int   currentBounceWall;

extern void   initBTCtrl(void);
extern void   periodicBTCtrl(void);

extern int    inArena(int preCheck, float *P);
extern float  calculateBounceHeading(int segmentIndex, float *P, float angle);
extern float  intersect2(float *P0, float *P1, float *P, float angle, float *Ps, float  *ts);
#endif
