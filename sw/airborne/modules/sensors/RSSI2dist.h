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
 * @file "modules/sensors/RSSI2dist.h"
 * @author T. Szabo
 * Estimate relative distance and velocity based on RSSI measurements from a BlueGiga Bluetooth dongle
 */

#ifndef RSSI2DIST_H
#define RSSI2DIST_H

#define VEC2_ASSIGN(_a, _x, _y) {		                      \
  (_a).v[0] = (_x);				                                \
  (_a).v[1] = (_y);				                                \
}
#define VEC2_CONST_PROD(_o, _v, _c) {                     \
  (_o).v[0] = (_v).v[0]*(_c);                             \
  (_o).v[1] = (_v).v[1]*(_c);                             \
}
#define VEC2_SUM(_o, _a, _b) {		                        \
  (_o).v[0] = (_a).v[0] + (_b).v[0];				              \
  (_o).v[1] = (_a).v[1] + (_b).v[1];				              \
}
#define VEC2_DIF(_o, _a, _b) {		                        \
  (_o).v[0] = (_a).v[0] - (_b).v[0];				              \
  (_o).v[1] = (_a).v[1] - (_b).v[1];				              \
}
#define RVEC2_CVEC2_PROD(_o, _a, _b) {                    \
  (_o) = (_a).v[0]*(_b).v[0] + (_a).v[1]*(_b).v[1];       \
}
#define CVEC2_RVEC2_PROD(_o, _a, _b) {                    \
  (_o).m[0] = (_a).v[0]*(_b).v[0];                        \
  (_o).m[1] = (_a).v[1]*(_b).v[0];                        \
  (_o).m[2] = (_a).v[0]*(_b).v[1];                        \
  (_o).m[3] = (_a).v[1]*(_b).v[1];                        \
}
#define MAT22_ASSIGN(_o,_a11,_a12, _a21, _a22) {          \
  (_o).m[0] = (_a11);                                     \
  (_o).m[1] = (_a21);                                     \
  (_o).m[2] = (_a12);                                     \
  (_o).m[3] = (_a22);                                     \
}
#define MAT22_ONES(_o) {                                  \
  (_o).m[0] = 1 ;				                                  \
  (_o).m[1] = 1 ;				                                  \
  (_o).m[2] = 1 ;				                                  \
  (_o).m[3] = 1 ;				                                  \
}
#define MAT22_ZEROS(_o) {                                 \
  (_o).m[0] = 0 ;				                                  \
  (_o).m[1] = 0 ;				                                  \
  (_o).m[2] = 0 ;				                                  \
  (_o).m[3] = 0 ;				                                  \
}
#define MAT22_EYE(_o) {                                   \
  (_o).m[0] = 1 ;				                                  \
  (_o).m[1] = 0 ;				                                  \
  (_o).m[2] = 0 ;				                                  \
  (_o).m[3] = 1 ;				                                  \
}
#define MAT22_SUM(_o, _a, _b) {		                        \
  (_o).m[0] = (_a).m[0] + (_b).m[0];				              \
  (_o).m[1] = (_a).m[1] + (_b).m[1];				              \
  (_o).m[2] = (_a).m[2] + (_b).m[2];				              \
  (_o).m[3] = (_a).m[3] + (_b).m[3];				              \
}
#define MAT22_DIF(_o, _a, _b) {		                        \
  (_o).m[0] = (_a).m[0] - (_b).m[0];				              \
  (_o).m[1] = (_a).m[1] - (_b).m[1];				              \
  (_o).m[2] = (_a).m[2] - (_b).m[2];				              \
  (_o).m[3] = (_a).m[3] - (_b).m[3];				              \
}
#define MAT22_MAT22_PROD(_o, _a, _b) {                    \
  (_o).m[0] = (_a).m[0]*(_b).m[0] + (_a).m[2]*(_b).m[1];  \
  (_o).m[1] = (_a).m[1]*(_b).m[0] + (_a).m[3]*(_b).m[1];  \
  (_o).m[2] = (_a).m[0]*(_b).m[2] + (_a).m[2]*(_b).m[3];  \
  (_o).m[3] = (_a).m[1]*(_b).m[2] + (_a).m[3]*(_b).m[3];  \
}
#define MAT22_CVEC2_PROD(_o, _a, _b) {                    \
  (_o).v[0] = (_a).m[0]*(_b).v[0] + (_a).m[2]*(_b).v[1];  \
  (_o).v[1] = (_a).m[1]*(_b).v[0] + (_a).m[3]*(_b).v[1];  \
}
#define RVEC2_MAT22_PROD(_o, _a, _b) {                    \
  (_o).v[0] = (_b).m[0]*(_a).v[0] + (_b).m[1]*(_a).v[1];  \
  (_o).v[1] = (_b).m[2]*(_a).v[0] + (_b).m[3]*(_a).v[1];  \
}
#define MAT22_TRANSP(_o, _a) {                            \
  (_o).m[0] = (_a.m)[0];                                  \
  (_o).m[1] = (_a).m[2];                                  \
  (_o).m[2] = (_a).m[1];                                  \
  (_o).m[3] = (_a).m[3];                                  \
}
#define MAT22_DET(_o, _a) {                               \
  (_o) = (_a).m[0]*(_a).m[3] - (_a).m[1]*(_a).m[2];       \
}
#define MAT22_CONST_PROD(_o, _m, _c) {                    \
  (_o).m[0] = (_m).m[0] * (_c);                           \
  (_o).m[1] = (_m).m[1] * (_c);                           \
  (_o).m[2] = (_m).m[2] * (_c);                           \
  (_o).m[3] = (_m).m[3] * (_c);                           \
}
#define MAT22_INV(_o, _a) {                               \
  double MAT_DET; MAT22_DET(MAT_DET,(_a));                \
  (_o).m[0] =  (_a).m[3]/MAT_DET;                         \
  (_o).m[1] = -(_a).m[1]/MAT_DET;                         \
  (_o).m[2] = -(_a).m[2]/MAT_DET;                         \
  (_o).m[3] =  (_a).m[0]/MAT_DET;                         \
}
#define MAT22_PROD_MAT22_PROD_MAT22_TRANSPOSE(_o, _a, _b, _c) {  \
  (_o).m[0] = (_c).m[0]*((_a).m[0]*(_b).m[0] + (_a).m[2]*(_b).m[1]) + (_c).m[2]*((_a).m[0]*(_b).m[2] + (_a).m[2]*(_b).m[3]); \
  (_o).m[0] = (_c).m[0]*((_a).m[1]*(_b).m[0] + (_a).m[3]*(_b).m[1]) + (_c).m[2]*((_a).m[1]*(_b).m[2] + (_a).m[3]*(_b).m[3]); \
  (_o).m[0] = (_c).m[1]*((_a).m[0]*(_b).m[0] + (_a).m[2]*(_b).m[1]) + (_c).m[3]*((_a).m[0]*(_b).m[2] + (_a).m[2]*(_b).m[3]); \
  (_o).m[0] = (_c).m[1]*((_a).m[1]*(_b).m[0] + (_a).m[3]*(_b).m[1]) + (_c).m[3]*((_a).m[1]*(_b).m[2] + (_a).m[3]*(_b).m[3]); \
}
#define MAT22_PRINT(_o) {                               \
  printf("[%.3f, %.3f]\n[%.3f, %.3f]\n",(_o).m[0],(_o).m[2],(_o).m[1],(_o).m[3]); \ 
}

#define RSSI_FSL_A                          -67
#define RSSI_FSL_n                          2.63
//#define RSSI_FSL_A                          -62
//#define RSSI_FSL_n                          3.2

#define RSSI_DIST_SENSOR_SATURATION_RANGE   5

typedef struct FloatVec2Struct
{
  double v[2];
} FloatVec2;
typedef struct FloatMat22Struct
{
  double m[4];
} FloatMat22;

FloatVec2   rssiEkf_estimate;

extern signed char rssi[];
extern char k_rssi;


extern void RSSI2Dist_init(void);
extern void RSSI2Dist_periodic(void);
extern void RSSI2Dist_event(void);

#endif
