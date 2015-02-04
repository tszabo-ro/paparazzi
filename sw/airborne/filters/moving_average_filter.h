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
 * @file "filters/moving_average_filter.h"
 * @author T. Szabo
 * Moving Average Filter
 */
 
#ifndef MOVING_AVERAGE_FILTER
#define MOVING_AVERAGE_FILTER

#include "inttypes.h"
#include <stddef.h>

typedef struct float_moving_average_filterStruct
{
  size_t  bufferSize;
  float  *buffer;
  uint8_t bufferPos;
  
  float   filterSum;
} float_moving_average_filter;



extern void     float_moving_average_filter_init(float_moving_average_filter *filter, size_t size, float defaultValue);
extern float    float_moving_average_filter_step(float_moving_average_filter *filter, float input);
extern void     float_moving_average_filter_cleanup(float_moving_average_filter *filter);
#endif
