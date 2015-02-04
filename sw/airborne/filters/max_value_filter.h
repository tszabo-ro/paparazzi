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
 * @file "filters/max_value_filter.h"
 * @author T. Szabo
 * Buffered maximum value filter: Outputs the highest value in the current buffer
 */
 
#ifndef MAX_VALUE_FILTER
#define MAX_VALUE_FILTER

#include "inttypes.h"
#include <stddef.h>

typedef struct int8_max_value_filterStruct
{
  size_t  bufferSize;
  int8_t  *buffer;
  uint8_t bufferPos;
} int8_max_value_filter;



extern void int_max_value_filter_init(int8_max_value_filter *filter, size_t size);
extern int8_t int_max_value_filter_step(int8_max_value_filter *filter, int8_t input);
extern void int_max_value_filter_cleanup(int8_max_value_filter *filter);
#endif
