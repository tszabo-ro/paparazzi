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
 * @file "filters/max_value_filter.c"
 * @author T. Szabo
 * Buffered maximum value filter: Outputs the highest value in the current buffer
 */
 
#include "max_value_filter.h"
#include <stdlib.h>
#include <limits.h>
 
void int_max_value_filter_init(int8_max_value_filter *filter, size_t size)
{
  filter->bufferSize  = size;
  filter->buffer      = malloc(filter->bufferSize*sizeof(int8_t));
  filter->bufferPos   = 0;
  
  for (uint8_t i = 0; i < filter->bufferSize; ++i)
    filter->buffer[i] = SCHAR_MIN;
}
int8_t int_max_value_filter_step(int8_max_value_filter *filter, int8_t input)
{
  filter->buffer[filter->bufferPos] = input;
  ++filter->bufferPos;
  if (filter->bufferPos >= filter->bufferSize)
    filter->bufferPos = 0;
    
  int8_t filterOutput = SCHAR_MIN;
  for (uint8_t i=0; i < filter->bufferSize; ++i)
  {
    if (filter->buffer[i] > filterOutput)
      filterOutput = filter->buffer[i];
  }
  return filterOutput;
}
void int_max_value_filter_cleanup(int8_max_value_filter *filter)
{
  free(filter->buffer);
  filter->bufferSize = 0;
  filter->bufferPos = 0;
  filter->buffer    = NULL;
}
