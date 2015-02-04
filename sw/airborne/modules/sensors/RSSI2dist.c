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

#include "modules/com/bluegiga.h"
#include "modules/sensors/RSSI2dist.h"
#include "filters/max_value_filter.h"
#include "filters/moving_average_filter.h"
#include "math.h"
#include <limits.h>

int8_max_value_filter       rssiInputFilter;
float_moving_average_filter rssiDistDiffFilter;

FloatVec2  rssiEkf_State;
FloatMat22 rssiEkf_PHI, rssiEkf_GAM, rssiEkf_P, rssiEkf_Q;
float rssiEkf_R;
  
void RSSI2Dist_init(void)
{

  int_max_value_filter_init(&rssiInputFilter,10);
  float_moving_average_filter_init(&rssiDistDiffFilter, 5, 0);

  // Initialize the rssi2distance EKF model
  VEC2_ASSIGN(rssiEkf_estimate, 2, 0);              // Initial state estimate
  VEC2_ASSIGN(rssiEkf_State, 2, 0);                 // Initial state estimate
  MAT22_ASSIGN(rssiEkf_PHI, 1   , (1/20), 0, 1);    // State estimate transition matrix
  MAT22_ASSIGN(rssiEkf_GAM, 0.01, 0     , 0, 0.01); // State noise input covariance matrix
  MAT22_ASSIGN(rssiEkf_Q  , 1   , 0     , 0, 1);    // Q
  MAT22_ASSIGN(rssiEkf_P  , 1   , 0     , 0, 0.01); // P
  
  rssiEkf_R = 100;
}
void RSSI2Dist_periodic(void)
{

  // Get the highest RSSI reading from the RSSI buffer
  signed char cRSSI = SCHAR_MIN;
  for (uint8_t i=0; i < 8; ++i)
  {
    if (rssi[i] > cRSSI)
      cRSSI = rssi[i];
  }
  
  // Get step the max value RSSI filter
  float cReading = (float)int_max_value_filter_step(&rssiInputFilter, cRSSI);

  // Save the last distance estimate for later
  float lastDistanceEstimate = rssiEkf_State.v[0];
  
  // BEGIN: RSSI-Distance model based EKF for estimating the relative distance & velocity
  float h_kp1_k = RSSI_FSL_A - 10*RSSI_FSL_n*log10(rssiEkf_State.v[0]);
  
  FloatMat22 tmpM1, tmpM2, P_kp1_k;
  MAT22_PROD_MAT22_PROD_MAT22_TRANSPOSE(tmpM1,rssiEkf_PHI,rssiEkf_P,rssiEkf_PHI);
  MAT22_PROD_MAT22_PROD_MAT22_TRANSPOSE(tmpM2,rssiEkf_GAM,rssiEkf_Q,rssiEkf_GAM);
  MAT22_SUM(P_kp1_k,tmpM1,tmpM2);
  
  FloatVec2 Hx_est, tmpV1;
  VEC2_ASSIGN(Hx_est, (-(10*RSSI_FSL_n)/(rssiEkf_State.v[0]*log(10))), 0);

  float Ve;
  RVEC2_MAT22_PROD(tmpV1, Hx_est, P_kp1_k);
  RVEC2_CVEC2_PROD(Ve,tmpV1,Hx_est); // Note: RVEC2_CVEC2_PROD makes no diffference btw. column & row vectors
  Ve += rssiEkf_R;

  FloatVec2 K;
  MAT22_CVEC2_PROD(tmpV1,P_kp1_k,Hx_est);
  VEC2_CONST_PROD(K,tmpV1, (1/Ve));

  // State update
  VEC2_CONST_PROD(tmpV1, K, (cReading - h_kp1_k));
  VEC2_SUM(rssiEkf_State, rssiEkf_State, tmpV1);
    
  // Covariance update
  MAT22_EYE(tmpM1);
  CVEC2_RVEC2_PROD(tmpM2,K,Hx_est);
  MAT22_DIF(tmpM1,tmpM1,tmpM2);
  
  MAT22_PROD_MAT22_PROD_MAT22_TRANSPOSE(tmpM2,tmpM1,P_kp1_k,tmpM1);
  CVEC2_RVEC2_PROD(tmpM1,K,K);
  MAT22_CONST_PROD(tmpM1, tmpM2, rssiEkf_R);
  
  MAT22_SUM(rssiEkf_P, tmpM1, tmpM2);
  //END: RSSI-Distance model based EKF for estimating the relative distance & velocity
  
  // Estimate the relative velocity via finite-difference & Step the moving average filter
  float fdRelVelEst = float_moving_average_filter_step(&rssiDistDiffFilter,((rssiEkf_State.v[0] - lastDistanceEstimate)/0.05));
  
  // Save the estimated distance & velocity
  rssiEkf_estimate.v[0] = rssiEkf_State.v[0]/RSSI_DIST_SENSOR_SATURATION_RANGE;
  rssiEkf_estimate.v[1] = (fdRelVelEst + (rssiEkf_State.v[0]/(RSSI_DIST_SENSOR_SATURATION_RANGE*RSSI_DIST_SENSOR_SATURATION_RANGE)))/2;
}
void RSSI2Dist_event(void)
{
}
