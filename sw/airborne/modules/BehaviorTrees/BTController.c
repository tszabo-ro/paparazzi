#include "BTController.h"
#include "BTStructure.h"
#include "../sensors/RSSI2dist.h"

#include "../../state.h"                                    // For stateGetPositionENU_i()
#include "../../math/pprz_algebra_int.h"
#include "../../firmwares/rotorcraft/guidance/guidance_h.h" // behavior_tree_control & the speed setpoints are defined here
#include "../../firmwares/rotorcraft/navigation.h"          // nav_heading is defined here!
#include "../../math/pprz_algebra.h"
#include "../../math/pprz_algebra_float.h"


BehaviorTree theBehaviorTree;
BTWorkspace  theBehaviorTreeWorkspace;

int lastBounceWall;

void initBTCtrl(void)
{
  // Initialize psiCmd as the current heading
  psiCmd = 100;
  
  /*
  float PCurrent[2];
  struct NedCoor_i *currentPos = stateGetPositionNed_i();
  PCurrent[0] = POS_FLOAT_OF_BFP(currentPos->x);
  PCurrent[1] = POS_FLOAT_OF_BFP(currentPos->y);*/
  currentBounceWall = -1;//inArena(-1, (float*)&PCurrent);
  
  // Initialize the BT
  initBT(&theBehaviorTree);
}



// Periodic function
void periodicBTCtrl(void)
{
  if (psiCmd == 100)
  {
    psiCmd = ANGLE_FLOAT_OF_BFP(nav_heading);
    FLOAT_ANGLE_NORMALIZE(psiCmd);
  }

  float PCurrent[2];
  struct NedCoor_f *currentPos = stateGetPositionNed_f();
  PCurrent[0] = currentPos->x;
  PCurrent[1] = currentPos->y;
  
  // Set PCurrent to the current ENU coordinates!

  //Check if we are still in the arena
  currentBounceWall = inArena(lastBounceWall, (float*)&PCurrent);
  
  // We are in the arena all right
  if (currentBounceWall < 0)
  {
    // Find the estimate to the closest drone
    float closestDistance = 100;
    for (int i=0; i < RSSI_DIST_SENSOR_MAX_NUM_TRACKED; ++i)
    {
      if (rssiDistEstimates[i] < closestDistance)
        closestDistance = rssiDistEstimates[i];
    }
    
    // Set the BT inputs
    theBehaviorTreeWorkspace.wpData[0] = closestDistance/5;   // Scaling required by BT
    if (theBehaviorTreeWorkspace.wpData[0] > 1)
      theBehaviorTreeWorkspace.wpData[0] = 1;
    
    
    // Tick the BT
    tickBT(&theBehaviorTree, &theBehaviorTreeWorkspace);
    
    // Get BT outputs
    speedCmd            = theBehaviorTreeWorkspace.wpData[BTWORKSPACE_NUM_INS + 0];
    psiDotCmd           = theBehaviorTreeWorkspace.wpData[BTWORKSPACE_NUM_INS + 1];

    dummy1 = theBehaviorTreeWorkspace.wpData[0];
    dummy2 = theBehaviorTreeWorkspace.wpData[1];
    dummy3 = theBehaviorTreeWorkspace.wpData[2];
    
    if (speedCmd > 1)
      speedCmd = 1;
    if (speedCmd < -1)
      speedCmd = -1;
      
    speedCmd = speedCmd*DRONE_MAX_VCMD;
    
    if (psiDotCmd > 1)
      psiDotCmd = 1;
    if (psiDotCmd < -1)
      psiDotCmd = -1;
      
    psiDotCmd = (-1)*psiDotCmd*DRONE_MAX_PSIDOT;
    
    psiCmd += (psiDotCmd*BTCONTROLLER_DT);
    
    while (psiCmd > M_PI)
      psiCmd -= 2*M_PI;
    while (psiCmd < -M_PI)
      psiCmd += 2*M_PI;
    
  }
  // We are outside of the arena, set the bounceback heading
  else
  {
    if (lastBounceWall != currentBounceWall)
      psiCmd = calculateBounceHeading(currentBounceWall, PCurrent, psiCmd);

    speedCmd          = 0.1f;
  }
  
/*  if (psiCmd > M_PI)
    psiCmd -= 2*M_PI;
  if (psiCmd < M_PI)
    psiCmd += 2*M_PI;
*/
  if (behavior_tree_control)
  {
    float hSP = psiCmd;
    while (hSP < 0)
      hSP += 2*M_PI;
    while (hSP > 2*M_PI)
      hSP -= 2*M_PI;
      
    nav_heading = ANGLE_BFP_OF_REAL(hSP);
    
    float vCmdX = cos(psiCmd)*speedCmd;
    float vCmdY = sin(psiCmd)*speedCmd;
    
      
    bt_speed_sp_i.x  = SPEED_BFP_OF_REAL(vCmdX);
    bt_speed_sp_i.y  = SPEED_BFP_OF_REAL(vCmdY);
  }
  else
  {
    psiCmd = 100;
    bt_speed_sp_i.x  = 0;//SPEED_BFP_OF_REAL(vCmdX);
    bt_speed_sp_i.y  = 0;//SPEED_BFP_OF_REAL(vCmdY);
  }
  
  lastBounceWall = currentBounceWall;
}
float calculateBounceHeading(int segmentIndex, float *P, float angle)
{
//  float impactPoint[2];
//  float ts;
//  float P0[2];
//  float P1[2];
  
  struct FloatVect2 N;
  
  if (segmentIndex == 0)
  {
//    P0[0]   = AREALIM_P0_X;
//    P0[1]   = AREALIM_P0_Y;    
//    P1[0]   = AREALIM_P1_X;
//    P1[1]   = AREALIM_P1_Y;
    N.x     = AL_N0_X;
    N.y     = AL_N0_Y;    
  }
  else if (segmentIndex == 1)
  {
//    P0[0]   = AREALIM_P1_X;
//    P0[1]   = AREALIM_P1_Y;    
//    P1[0]   = AREALIM_P2_X;
//    P1[1]   = AREALIM_P2_Y;
    N.x     = AL_N1_X;
    N.y     = AL_N1_Y;
  }
  else if (segmentIndex == 2)
  {
//    P0[0]   = AREALIM_P2_X;
//    P0[1]   = AREALIM_P2_Y;    
//    P1[0]   = AREALIM_P3_X;
//    P1[1]   = AREALIM_P3_Y;
    N.x     = AL_N2_X;
    N.y     = AL_N2_Y;
  }
  else if (segmentIndex == 3)
  {
//    P0[0]   = AREALIM_P3_X;
//    P0[1]   = AREALIM_P3_Y;    
//    P1[0]   = AREALIM_P0_X;
//    P1[1]   = AREALIM_P0_Y;
    N.x     = AL_N3_X;
    N.y     = AL_N3_Y;
  }
  else
  { return angle; }
  
  struct FloatVect2 V,R;
  V.x = cos(angle);
  V.y = sin(angle);
  
  float N_norm;
  FLOAT_VECT2_NORM(N_norm, N);
  N.x /= N_norm;
  N.y /= N_norm;
  
  // R = -2*N*dot(V,N) + V, where: V-incidence vector, N-plane normal vector, R-bounce vector
  float s = -2*FLOAT_VECT2_DOT_PRODUCT(V,N);
  FLOAT_VECT2_SMUL(R,N,s);
  FLOAT_VECT2_ADD(R,V);
  
  return atan2(R.y,R.x);
}
int inArena(int preCheck, float *P)
{
  float X = P[0];
  float Y = P[1];



  float tmp;

  int wI = -100;
  if (preCheck == 0)
  {
    tmp = (AREALIM_P1_X - AREALIM_P0_X)*(Y - AREALIM_P0_Y) - (AREALIM_P1_Y - AREALIM_P0_Y)*(X - AREALIM_P0_X);
    if ( tmp WALL_THREASHOLD 1)
      { wI = 0; }
  }
  else if (preCheck == 1)
  {
    tmp = (AREALIM_P2_X - AREALIM_P1_X)*(Y - AREALIM_P1_Y) - (AREALIM_P2_Y - AREALIM_P1_Y)*(X - AREALIM_P1_X);
    if ( tmp WALL_THREASHOLD 1)
      { wI = 1; }
  }
  else if (preCheck == 2)
  {
    tmp = (AREALIM_P3_X - AREALIM_P2_X)*(Y - AREALIM_P2_Y) - (AREALIM_P3_Y - AREALIM_P2_Y)*(X - AREALIM_P2_X);
    if ( tmp WALL_THREASHOLD 1)
      { wI = 2; }
  }
  else if (preCheck == 3)
  {
    tmp = (AREALIM_P0_X - AREALIM_P3_X)*(Y - AREALIM_P3_Y) - (AREALIM_P0_Y - AREALIM_P3_Y)*(X - AREALIM_P3_X);
    if ( tmp WALL_THREASHOLD 1)
      { wI = 3; }
  }
  else
  {
    tmp = (AREALIM_P1_X - AREALIM_P0_X)*(Y - AREALIM_P0_Y) - (AREALIM_P1_Y - AREALIM_P0_Y)*(X - AREALIM_P0_X);
    if ( tmp WALL_THREASHOLD 1)
      { wI = 0; }
      
    tmp = (AREALIM_P2_X - AREALIM_P1_X)*(Y - AREALIM_P1_Y) - (AREALIM_P2_Y - AREALIM_P1_Y)*(X - AREALIM_P1_X);
    if ( tmp WALL_THREASHOLD 1)
      { wI = 1; }
      
    tmp = (AREALIM_P3_X - AREALIM_P2_X)*(Y - AREALIM_P2_Y) - (AREALIM_P3_Y - AREALIM_P2_Y)*(X - AREALIM_P2_X);
    if ( tmp WALL_THREASHOLD 1)
      { wI = 2; }
      
    tmp = (AREALIM_P0_X - AREALIM_P3_X)*(Y - AREALIM_P3_Y) - (AREALIM_P0_Y - AREALIM_P3_Y)*(X - AREALIM_P3_X);
    if ( tmp WALL_THREASHOLD 1)
      { wI = 3; }
  }
  
  return wI;
}
