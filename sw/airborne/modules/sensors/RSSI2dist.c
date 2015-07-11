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

//#include "modules/com/bluegiga.h"
#include "modules/sensors/RSSI2dist.h"
#include "filters/max_value_filter.h"
#include "filters/moving_average_filter.h"
#include "math.h"
#include <limits.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/time.h>

int8_max_value_filter       rssiInputFilter[RSSI_DIST_SENSOR_MAX_NUM_TRACKED];
//float_moving_average_filter rssiDistDiffFilter[RSSI_DIST_SENSOR_MAX_NUM_TRACKED];

signed char rssi[8];
signed char prefiltRSSI[8];
char k_rssi = 0;

int sock, bytes_recv, sin_size;
struct sockaddr_in server_addr;
struct hostent *host;
char send_data[1024], recv_data[1024];

//FloatVec2  rssiEkf_State;
//FloatMat22 rssiEkf_PHI, rssiEkf_GAM, rssiEkf_P, rssiEkf_Q;


unsigned long long lastTime;

long rssiSum;
unsigned int sumCount;

void RSSI2Dist_init(void)
{
/*
// Set up the filters
  int_max_value_filter_init(&rssiInputFilter,10);
  float_moving_average_filter_init(&rssiDistDiffFilter, 5, 0);


  // Initialize the rssi2distance EKF model
  VEC2_ASSIGN(rssiEkf_estimate, 2, 0);              // Initial state estimate
  VEC2_ASSIGN(rssiEkf_State, 1, 0);                 // Initial state estimate
  MAT22_ASSIGN(rssiEkf_PHI, 1,  (1.f/20.f),0,  1);     // State estimate transition matrix
  MAT22_ASSIGN(rssiEkf_GAM, 0.01, 0     , 0, 0.01);  // State noise input covariance matrix
  MAT22_ASSIGN(rssiEkf_Q  , 1   , 0     , 0, 100);   // Q
  MAT22_ASSIGN(rssiEkf_P  , 1   , 0     , 0, 0.01); // P
  
//  rssiEkf_PHI.m[0] = 1;   rssiEkf_PHI.m[1] = 0; rssiEkf_PHI.m[2] = 0.05f;  rssiEkf_PHI.m[3] = 1; 
//  rssiEkf_GAM.m[0] = 0.1; rssiEkf_GAM.m[1] = 0; rssiEkf_GAM.m[2] = 0;       rssiEkf_GAM.m[3] = 0.1; 
//  rssiEkf_Q.m[0] = 1;   rssiEkf_Q.m[1] = 0; rssiEkf_Q.m[2] = 0;  rssiEkf_Q.m[3] = 10; 
//  rssiEkf_P.m[0] = 1;   rssiEkf_P.m[1] = 0; rssiEkf_P.m[2] = 0;  rssiEkf_P.m[3] = 0.1; 

  rssiEkf_R = 200;*/
  
  
  // Initialize RSSI EKF
  for (int i=0; i < RSSI_DIST_SENSOR_MAX_NUM_TRACKED; ++i)
  {
    rssiDistEstimates[i] = 5;
    rssiDistEstimatesP[i] = 10;
    int_max_value_filter_init(&rssiInputFilter[i],5);
  }
  
  
  lastTime = 0;
  
// Set up the UDP connection to the Bluegiga app
  host = (struct hostent *) gethostbyname((char *)"127.0.0.1");
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("socket");
    exit(1);
  }

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(5000);
  server_addr.sin_addr = *((struct in_addr *)host->h_addr);
  bzero(&(server_addr.sin_zero), 8);
  sin_size = sizeof(struct sockaddr);

  strcpy(send_data, "1");

  if (bind(sock, (struct sockaddr *)&server_addr,
           sizeof(struct sockaddr)) == -1) {
    perror("Bind");
    exit(1);
  }
  
  rssi[0] = SCHAR_MIN;
  rssi[1] = SCHAR_MIN;
  rssi[2] = SCHAR_MIN;
  rssi[3] = SCHAR_MIN;
  rssi[4] = SCHAR_MIN;
  rssi[5] = SCHAR_MIN;
  rssi[6] = SCHAR_MIN;
  rssi[7] = SCHAR_MIN;
}
void RSSI2Dist_periodic(void)
{
  char hasReceived = 0;
  
  // Receive data from the Bluegiga app
  bytes_recv = recvfrom(sock, recv_data, 1024, MSG_DONTWAIT, (struct sockaddr *)&server_addr, (socklen_t *)&sin_size);
  while (bytes_recv > 0)
  {
    hasReceived = 1;
    
    k_rssi = bytes_recv;
    for (int i = 0; i < k_rssi; i++) {
      rssi[i] = (signed char) recv_data[i];
    }
    bytes_recv = recvfrom(sock, recv_data, 1024, MSG_DONTWAIT, (struct sockaddr *)&server_addr, (socklen_t *)&sin_size);
  }
  if (hasReceived == 0)
    return;
    
  // Calculate the time from the last update
/*  struct timeval tv;
  gettimeofday(&tv,NULL);
  
  unsigned long long cTime = tv.tv_sec*1000000 + tv.tv_usec;
  double dt = (((double)(cTime-lastTime))/1000000);
//  MAT22_ASSIGN(rssiEkf_PHI, 1   , dt, 0, 1);
  
  lastTime = cTime;
*/
/*
  // Get the highest RSSI reading from the RSSI buffer
  signed char cRSSI = SCHAR_MIN;
  for (uint8_t i=0; i < 8; ++i)
  {
    if (rssi[i] > cRSSI)
      cRSSI = rssi[i];
  }
  
//  rssiSum += cRSSI;
//  sumCount++;
  // Get step the max value RSSI filter
  rssiFilt = (float)int_max_value_filter_step(&rssiInputFilter, cRSSI);
//  float rssiFilt = float_moving_average_filter_step(&rssiInputFilter, cRSSI);
  
  // Save the last distance estimate for later
  float lastDistanceEstimate = rssiEkf_State.v[0];
  
  // BEGIN: RSSI-Distance model based EKF for estimating the relative distance & velocity
  float h_kp1_k = RSSI_FSL_A - 10*RSSI_FSL_n*log10(rssiEkf_State.v[0]); // h_kp1_k = A - 10n*log10(dist)
  
  FloatMat22 tmpM1, tmpM2, tmpM3, tmpM4, P_kp1_k;
  
  //MAT22_PROD_MAT22_PROD_MAT22_TRANSPOSE(tmpM1,rssiEkf_PHI,rssiEkf_P,rssiEkf_PHI);
  MAT22_TRANSP(tmpM1,rssiEkf_PHI);                // tmpM1 = transp(PHI)
  MAT22_MAT22_PROD(tmpM2,rssiEkf_PHI, rssiEkf_P); // tmpM2 = PHI*P
  MAT22_MAT22_PROD(tmpM3,tmpM2,tmpM1);            // tmpM3 = tmpM2*tmpM1 = PHI*P*transp(PHI)
  
  //MAT22_PROD_MAT22_PROD_MAT22_TRANSPOSE(tmpM2,rssiEkf_GAM,rssiEkf_Q,rssiEkf_GAM);
  MAT22_TRANSP(tmpM1,rssiEkf_GAM);                // tmpM1 = transp(GAM)
  MAT22_MAT22_PROD(tmpM2,rssiEkf_GAM, rssiEkf_Q); // tmpM2 = GAM*Q
  MAT22_MAT22_PROD(tmpM4,tmpM2,tmpM1);            // tmpM4 = tmpM2*tmpM1 = GAM*Q*transp(GAM)
  
  MAT22_SUM(P_kp1_k,tmpM3,tmpM4);                 // P_kp1_k = tmpM3 + tmpM4 = PHI*P*transp(PHI) + GAM*Q*transp(GAM)
  
  FloatVec2 Hx_est, tmpV1;
  VEC2_ASSIGN(Hx_est, (-(10*RSSI_FSL_n)/(rssiEkf_State.v[0]*log(10))), 0); // Estimate Hx as [-(10n)/(d*log(10)), 0]

  float Ve;
  // Note: The vector macros make no difference between row or column vectors!
  RVEC2_MAT22_PROD(tmpV1, Hx_est, P_kp1_k);       // tmpV1 = Hx_est*P_kp1_k
  RVEC2_CVEC2_PROD(Ve,tmpV1,Hx_est);              // Ve = tmpV1*transp(H_est) = Hx_est*P_kp1_k*transp(Hx_est);
  Ve += rssiEkf_R;                                // Ve = Ve + R = Hx_est*P_kp1_k*transp(Hx_est) + R

  FloatVec2 K;
  MAT22_CVEC2_PROD(tmpV1,P_kp1_k,Hx_est);         // tmpV1 = P_kp1_k*transp(Hx_est)
  VEC2_CONST_PROD(K,tmpV1, (1/Ve));               // K = tmpV1*inv(Ve) = (P_kp1_k*transp(Hx_est))*inv(Ve)

  // State update
  VEC2_CONST_PROD(tmpV1, K, (rssiFilt - h_kp1_k));// tmpV1 = K*(rssiFilt - h_kp1_k)
  VEC2_SUM(rssiEkf_State, rssiEkf_State, tmpV1);  // x_kp1_kp1 = x_k_k + tmpV1 = x_kp1_k + K*(rssiFilt - h_kp1_k)
    
  // Covariance update
  MAT22_EYE(tmpM1);                               // tmpM1 = I2
  CVEC2_RVEC2_PROD(tmpM2,K,Hx_est);               // tmpM2 = K*Hx_est
  MAT22_DIF(tmpM3,tmpM1,tmpM2);                   // tmpM3 = tmpM1 - tmpM2 = I2 - K*Hx_est

  MAT22_PROD_MAT22_PROD_MAT22_TRANSPOSE(tmpM2,tmpM1,P_kp1_k,tmpM1);  
  MAT22_TRANSP(tmpM4, tmpM3);                     // tmpM4 = transp(tmpM3) = transp(I2 - K*Hx_est)
  MAT22_MAT22_PROD(tmpM1, tmpM3, P_kp1_k);        // tmpM1 = tmpM3*P_kp1_k = (I2 - K*Hx_est)*P_kp1_k
  MAT22_MAT22_PROD(tmpM2, tmpM1, tmpM4);          // tmpM2 = tmpM1*tmpM4   = (I2 - K*Hx_est)*P_kp1_k*transp(I2 - K*Hx_est)
  
  VEC2_CONST_PROD(tmpV1, K, rssiEkf_R);           // tmpV1 = K*R
  CVEC2_RVEC2_PROD(tmpM1, tmpV1, K);              // tmpM1 = tmpV1*transp(K);
  
  MAT22_SUM(rssiEkf_P, tmpM1, tmpM2);             // P = tmpM1+tmpM2 = K*R*transp(K) + (I2-K*Hx_est)*P_kp1_k*transp(I2-K*Hx_est)
  //END: RSSI-Distance model based EKF for estimating the relative distance & velocity
  
  // Estimate the relative velocity via finite-difference & Step the moving average filter
  fdRelVelEst = float_moving_average_filter_step(&rssiDistDiffFilter,((rssiEkf_State.v[0] - lastDistanceEstimate)/0.05));
  
  // Save the estimated distance & velocity
  rssiEkf_estimate.v[0] = rssiEkf_State.v[0]/RSSI_DIST_SENSOR_SATURATION_RANGE;
  rssiEkf_estimate.v[1] = (fdRelVelEst + (rssiEkf_State.v[1]))/2;
  
  dEstFilt = pow(10,(RSSI_FSL_A - rssiFilt)/(10*RSSI_FSL_n));
  dEstRaw = pow(10,(RSSI_FSL_A - cRSSI)/(10*RSSI_FSL_n));*/
  /*printf("T: %.3fs - RSSI: %d(%d) / dEst: %.3f (%.3f / %.3f)/ vEst: KF: %.3f/Diff: %.3f/M: %.3f\n", 
    dt, 
    (int)rssiFilt, 
    cRSSI,
    rssiEkf_State.v[0],
    dEstRaw,
    dEstFilt,
    rssiEkf_State.v[1],
    fdRelVelEst,
    rssiEkf_estimate.v[1]
    );*/
    
    
  // Estimate distances from RSSI readings
  printf("Stepping filters: \n");
  for (int i=0; i < RSSI_DIST_SENSOR_MAX_NUM_TRACKED; ++i)
  {
    prefiltRSSI[i] = (float)int_max_value_filter_step((int8_max_value_filter*)&rssiInputFilter[i], rssi[i]);
    
    // BEGIN: RSSI-Distance model based EKF for estimating the relative distance & velocity
    float h_kp1_k = RSSI_FSL_A - 10*RSSI_FSL_n*log10(rssiDistEstimates[i]); // h_kp1_k = A - 10n*log10(dist)
    float Hx      = (-(10*RSSI_FSL_n)/(rssiDistEstimates[i]*log(10)));
    
    float P_kp1_k = rssiDistEstimatesP[i] + RSSI_DIST_Q;
    float Ve      = Hx*P_kp1_k*Hx + RSSI_DIST_R;
    float K       = P_kp1_k*Hx/Ve;
        
    rssiDistEstimates[i] += K*(prefiltRSSI[i] - h_kp1_k);
    rssiDistEstimatesP[i] = (1-K*Hx)*P_kp1_k*(1-K*Hx) + K*RSSI_DIST_R*K;
  }
}
