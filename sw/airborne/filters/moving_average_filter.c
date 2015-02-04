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
 * @file "filters/moving_average_filter.c"
 * @author T. Szabo
 * Moving Average Filter
 */
 
#include "moving_average_filter.h"
#include <stdlib.h>
#include <limits.h>

void     float_moving_average_filter_init(float_moving_average_filter *filter, size_t size, float defaultValue)
{
  filter->bufferSize  = size;
  filter->buffer      = malloc(filter->bufferSize*sizeof(float));
  filter->bufferPos   = 0;
  
  filter->filterSum   = size*defaultValue;
  
  for (uint8_t i = 0; i < filter->bufferSize; ++i)
    filter->buffer[i] = defaultValue;
}
float   float_moving_average_filter_step(float_moving_average_filter *filter, float input)
{
  filter->filterSum                 += input - filter->buffer[filter->bufferPos];
  filter->buffer[filter->bufferPos]  = input;
  
  ++filter->bufferPos;
  if (filter->bufferPos >= filter->bufferSize)
    filter->bufferPos = 0;
    
  return (filter->filterSum / filter->bufferSize);
}
void     float_moving_average_filter_cleanup(float_moving_average_filter *filter)
{
  free(filter->buffer);
  filter->bufferSize = 0;
  filter->bufferPos = 0;
  filter->buffer    = NULL;
}
