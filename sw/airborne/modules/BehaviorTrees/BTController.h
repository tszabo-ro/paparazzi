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

#include "../../filters/low_pass_filter.h"
#include "../../math/pprz_algebra_float.h"

#define     DRONE_MAX_VCMD          0.4f
#define     DRONE_MAX_PSIDOT        (40*(M_PI/180))

#define     BTCONTROLLER_DT         0.05f
#define     GEOMETRY_SMALL_NUMBER   0.00000001f


// The walls are defined as P0->P1->P2->P3->P0 clockwise when looking at the arena from above. If the ordering is reversed, change WALL_THRESHOLD_OPERATOR to >
#define     WALL_THRESHOLD_OPERATOR   < 
#define     WALL_THRESHOLD          1
#define     WALL_CAUTION_THRESHOLD  2

// Bounce frequency limiter defined in number of cycles
#define     MIN_WALL_BOUNCE_TIME    10

#define     AREALIM_P0_X           -4.34f
#define     AREALIM_P0_Y           -0.87f

#define     AREALIM_P1_X            0.90f
#define     AREALIM_P1_Y           -4.29f

#define     AREALIM_P2_X            4.20f
#define     AREALIM_P2_Y            0.92f

#define     AREALIM_P3_X           -0.80f
#define     AREALIM_P3_Y            4.43f

//#define     AREALIM_P0_X           -3.91f
//#define     AREALIM_P0_Y           -0.78f

//#define     AREALIM_P1_X            0.81f
//#define     AREALIM_P1_Y           -3.86f

//#define     AREALIM_P2_X            3.78f
//#define     AREALIM_P2_Y            0.82f

//#define     AREALIM_P3_X           -0.72f
//#define     AREALIM_P3_Y            3.98f

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
float speedScale;

float btIO_0;
float btIO_1;
float btIO_2;
float btIO_3;

int   currentBounceWall;

extern void   initBTCtrl(void);
extern void   periodicBTCtrl(void);

extern int    inArena(int preCheck, float *P, float *heading, float *bounce);
extern float  intersect2(float *P0, float *P1, float *P, float angle, float *Ps, float  *ts);
#endif
