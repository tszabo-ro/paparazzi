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
 * @file "modules/monoVisionAvoid/monoVisionAvoid.h"
 * @author T. Szabo
 * Obstacle avoidance based on monocular vision
 */
#ifndef MONOVISIONAVOID_H
#define MONOVISIONAVOID_H

#include "math.h"
#include "../../math/pprz_algebra.h"
#include "../../math/pprz_algebra_int.h"

// Max turn rate of the drone - set to 30 deg/s for now
#define AVOID_MAX_TURNRATE    ((30)*(M_PI/180))

// Max Forward velocity of the drone - set to 1 m/s for now
#define AVOID_MAX_VELOCITY    1

// update rate of this module - used for integration. Set to 20 Hz (0.05 s)
#define MODULE_UPDATE_RATE    0.05f


extern void monoVisionAvoid_init(void);
extern void monoVisionAvoid_periodic(void);

extern bool monoVisionAvoid_flightPlanUpdate(void);
#endif
