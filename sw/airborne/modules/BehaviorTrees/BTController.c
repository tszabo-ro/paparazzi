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

uint8_t speedFlip;
uint16_t flipCount;
uint16_t flipCycle;

unsigned long  wallBounceTime; // Defined in number of samples!
bool           wallBounceEnabled; // Flag for preventing consecutive bounces from the same wall
void   calculateBounceHeading( struct FloatVect2 *P0, struct FloatVect2 *P1, struct FloatVect2 *P, float *heading, float *bounce);

void initBTCtrl(void)
{
   // Initialize psiCmd as the current heading
   psiCmd = 100;
  
   currentBounceWall = -1;//inArena(-1, (float*)&PCurrent);

   speedFlip = 1;  
   speedCmd  = 1;
   speedScale = 1;
   flipCycle = 1;
   flipCount = 0;

   wallBounceTime    = 0;
   wallBounceEnabled = true;
   
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
      speedCmd = DRONE_MAX_VCMD;
   }

   // Set PCurrent to the current ENU coordinates!
   float PCurrent[2];
   struct NedCoor_f *currentPos = stateGetPositionNed_f();
   PCurrent[0] = currentPos->x;
   PCurrent[1] = currentPos->y;

   float bounceHeading;

   //Check if we are still in the arena
   currentBounceWall = inArena(lastBounceWall, (float*)&PCurrent, &psiCmd, &bounceHeading);
  
   // We are in the arena all right
   if (currentBounceWall < -50)
   {
      // Find the estimate to the closest drone
      float closestDistance = 100;
      for (int i=0; i < RSSI_DIST_SENSOR_MAX_NUM_TRACKED; ++i)
      {
         if (rssiDistEstimates[i] < closestDistance)
            closestDistance = rssiDistEstimates[i];
      }
    
      // Set the BT inputs
      theBehaviorTreeWorkspace.wpData[0] = closestDistance/2.5-1;   // Scaling required by BT

      if (theBehaviorTreeWorkspace.wpData[0] > 1)
         theBehaviorTreeWorkspace.wpData[0] = 1;
      if (theBehaviorTreeWorkspace.wpData[0] < -1)
         theBehaviorTreeWorkspace.wpData[0] = -1;    
    
      // Tick the BT
      tickBT(&theBehaviorTree, &theBehaviorTreeWorkspace);
    
      // Get BT outputs
      speedCmd            = theBehaviorTreeWorkspace.wpData[BTWORKSPACE_NUM_INS + 0];
      psiDotCmd           = theBehaviorTreeWorkspace.wpData[BTWORKSPACE_NUM_INS + 1];
      float headingInc    = theBehaviorTreeWorkspace.wpData[BTWORKSPACE_NUM_INS + 2]*2*M_PI;

      btIO_0 = theBehaviorTreeWorkspace.wpData[0];
      btIO_1 = theBehaviorTreeWorkspace.wpData[1];
      btIO_2 = theBehaviorTreeWorkspace.wpData[2];
      btIO_3 = theBehaviorTreeWorkspace.wpData[3];
    
      if (speedCmd > 1)
         speedCmd = 1;
      if (speedCmd < -1)
         speedCmd = -1;
    
      if (psiDotCmd > 1)
         psiDotCmd = 1;
      if (psiDotCmd < -1)
         psiDotCmd = -1;
      
      psiDotCmd = psiDotCmd*DRONE_MAX_PSIDOT; 
      psiCmd += (psiDotCmd*BTCONTROLLER_DT) + headingInc;
    
      while (psiCmd > M_PI)
         psiCmd -= 2*M_PI;
      while (psiCmd < -M_PI)
         psiCmd += 2*M_PI; 
   }
   // We are outside of the arena, set the bounceback heading
   else
   {
      psiCmd   = bounceHeading;
   }
   if (behavior_tree_control)
   {
      float hSP = psiCmd;
      while (hSP < 0)
         hSP += 2*M_PI;
      while (hSP > 2*M_PI)
         hSP -= 2*M_PI;
      
      nav_heading = ANGLE_BFP_OF_REAL(hSP);
    
      float vCmdX = cos(psiCmd)*speedCmd*speedScale*DRONE_MAX_VCMD;
      float vCmdY = sin(psiCmd)*speedCmd*speedScale*DRONE_MAX_VCMD;
      
      bt_speed_sp_f.x  = vCmdX;
      bt_speed_sp_f.y  = vCmdY;
   }
   else
   {
      psiCmd = 100;
      bt_speed_sp_f.x  = 0;
      bt_speed_sp_f.y  = 0;
   }
   lastBounceWall = currentBounceWall;
}
int    inArena(int preCheck, float *P, float *heading, float *bounce)
{
   float X = P[0];
   float Y = P[1];

   *bounce    = *heading;

   

   int wI = -100;
   if (preCheck == 0)
   {
      if ( 
            (AREALIM_P1_X - AREALIM_P0_X)*(Y - AREALIM_P0_Y) - (AREALIM_P1_Y - AREALIM_P0_Y)*(X - AREALIM_P0_X)
             WALL_THRESHOLD_OPERATOR 
               WALL_THRESHOLD
         )
      {
         if ( (wallBounceTime++ > MIN_WALL_BOUNCE_TIME) && ( wallBounceEnabled ) )
         {
            struct FloatVect2 P0, P1, POS;
            P0.x = AREALIM_P0_X;
            P0.y = AREALIM_P0_Y;
            P1.x = AREALIM_P1_X;
            P1.y = AREALIM_P1_Y;
            POS.x = X;
            POS.y = Y;
            calculateBounceHeading( &P0, &P1, &POS, heading, bounce);
            wallBounceEnabled = false;
         }
            
         return 0;
      }
   }
   else if (preCheck == 1)
   {
      if ( 
            (AREALIM_P2_X - AREALIM_P1_X)*(Y - AREALIM_P1_Y) - (AREALIM_P2_Y - AREALIM_P1_Y)*(X - AREALIM_P1_X)
             WALL_THRESHOLD_OPERATOR 
               WALL_THRESHOLD
         )
      {
         if ( (wallBounceTime++ > MIN_WALL_BOUNCE_TIME) && ( wallBounceEnabled ) )
         {
            struct FloatVect2 P0, P1, POS;
            P0.x = AREALIM_P1_X;
            P0.y = AREALIM_P1_Y;
            P1.x = AREALIM_P2_X;
            P1.y = AREALIM_P2_Y;
            POS.x = X;
            POS.y = Y;
            calculateBounceHeading( &P0, &P1, &POS, heading, bounce);
            wallBounceEnabled = false;
         }
            
         return 1;
      }
   }
   else if (preCheck == 2)
   {
      if ( 
            (AREALIM_P3_X - AREALIM_P2_X)*(Y - AREALIM_P2_Y) - (AREALIM_P3_Y - AREALIM_P2_Y)*(X - AREALIM_P2_X)
             WALL_THRESHOLD_OPERATOR 
               WALL_THRESHOLD
         )
      {
         if ( (wallBounceTime++ > MIN_WALL_BOUNCE_TIME) && ( wallBounceEnabled ) )
         {
            struct FloatVect2 P0, P1, POS;
            P0.x = AREALIM_P2_X;
            P0.y = AREALIM_P2_Y;
            P1.x = AREALIM_P3_X;
            P1.y = AREALIM_P3_Y;
            POS.x = X;
            POS.y = Y;
            calculateBounceHeading( &P0, &P1, &POS, heading, bounce);
            wallBounceEnabled = false;
         }
            
         return 2;
      }
   }
   else if (preCheck == 3)
   {
      if ( 
            (AREALIM_P0_X - AREALIM_P3_X)*(Y - AREALIM_P3_Y) - (AREALIM_P0_Y - AREALIM_P3_Y)*(X - AREALIM_P3_X)
             WALL_THRESHOLD_OPERATOR 
               WALL_THRESHOLD
         )
      {
         if ( (wallBounceTime++ > MIN_WALL_BOUNCE_TIME) && ( wallBounceEnabled ) )
         {
            struct FloatVect2 P0, P1, POS;
            P0.x = AREALIM_P3_X;
            P0.y = AREALIM_P3_Y;
            P1.x = AREALIM_P0_X;
            P1.y = AREALIM_P0_Y;
            POS.x = X;
            POS.y = Y;
            calculateBounceHeading( &P0, &P1, &POS, heading, bounce);
            wallBounceEnabled = false;
         }
            
         return 3;
      }
   }

   if ( (preCheck != 0) && (preCheck != 1) && (preCheck != 2) && (preCheck != 3) )
   {
      float tmp;
      tmp = (AREALIM_P1_X - AREALIM_P0_X)*(Y - AREALIM_P0_Y) - (AREALIM_P1_Y - AREALIM_P0_Y)*(X - AREALIM_P0_X);
      if ( 
            tmp
             WALL_THRESHOLD_OPERATOR 
               WALL_THRESHOLD
         )
      { wI      = 0; wallBounceTime    = 0; wallBounceEnabled = true; }
      else if ( tmp WALL_THRESHOLD_OPERATOR WALL_CAUTION_THRESHOLD)
         { speedScale = 0.5; }
      
      tmp = (AREALIM_P2_X - AREALIM_P1_X)*(Y - AREALIM_P1_Y) - (AREALIM_P2_Y - AREALIM_P1_Y)*(X - AREALIM_P1_X);
      if ( 
            tmp
             WALL_THRESHOLD_OPERATOR 
               WALL_THRESHOLD
         )
      { wI      = 1; wallBounceTime    = 0; wallBounceEnabled = true; }
      else if ( tmp WALL_THRESHOLD_OPERATOR WALL_CAUTION_THRESHOLD)
         { speedScale = 0.5; }
      

      tmp = (AREALIM_P3_X - AREALIM_P2_X)*(Y - AREALIM_P2_Y) - (AREALIM_P3_Y - AREALIM_P2_Y)*(X - AREALIM_P2_X);
      if ( 
            tmp
             WALL_THRESHOLD_OPERATOR 
               WALL_THRESHOLD
         )
      { wI      = 2; wallBounceTime    = 0; wallBounceEnabled = true; }
      else if ( tmp WALL_THRESHOLD_OPERATOR WALL_CAUTION_THRESHOLD)
         { speedScale = 0.5; }
      
      tmp = (AREALIM_P0_X - AREALIM_P3_X)*(Y - AREALIM_P3_Y) - (AREALIM_P0_Y - AREALIM_P3_Y)*(X - AREALIM_P3_X);
      if ( 
            tmp
             WALL_THRESHOLD_OPERATOR 
               WALL_THRESHOLD
         )
      { wI      = 3; wallBounceTime    = 0; wallBounceEnabled = true; }
      else if ( tmp WALL_THRESHOLD_OPERATOR WALL_CAUTION_THRESHOLD)
         { speedScale = 0.5; }
   }

   if (wI < 0)
      {wallBounceTime    = 0; wallBounceEnabled = true; speedScale = 1; }

   return wI;
}
void   calculateBounceHeading( struct FloatVect2 *P0, struct FloatVect2 *P1, struct FloatVect2 *P, float *heading, float *bounce)
{
   // doubles are used here because at small incidence angles numerical issues appear with detecting the flight direction
   double Nx, Ny, Rx, Ry, Hx, Hy, Px, Py, P0x, P0y, P1x, P1y, P0P1x, P0P1y;

   Px  = P->x;
   Py  = P->y;
   P0x = P0->x;
   P0y = P0->y;
   P1x = P1->x;
   P1y = P1->y;

   // A large vector is used as pointing heading to limit numberical issues at small angles
   Hx = cos(*heading);
   Hy = sin(*heading);

   P0P1x = P1x-P0x;
   P0P1y = P1y-P0y;

   // Extend the wall segment
   P0x   = P0x - 10000*P0P1x;
   P0y   = P0y - 10000*P0P1y;
   P1x   = P1x + 10000*P0P1x;
   P1y   = P1y + 10000*P0P1y;

   // Normal of the wall
   Nx = (P1y - P0y);
   Ny = (P0x - P1x);

   // Make the length of the normal equal to that of H
   double Nmag = sqrt(Nx*Nx + Ny*Ny);
   Nx = (Nx/Nmag);
   Ny = (Ny/Nmag);


   // Check if the drone entered the wall flying back or forward
   bool backwardsFlight = true;   

   double tmp = (P1x - P0x)*((Py + (1000*Hy)) - P0y) - (P1y - P0y)*((Px+(1000*Hx)) - P0x);
   if (tmp WALL_THRESHOLD_OPERATOR 0)
      backwardsFlight = false;
   else
      backwardsFlight = true;

   if (backwardsFlight)
   {
      // If we're flying backwards, flip the (heading) pointing vector, so the bounce will happen the right way.
      Hx = (-1)*Hx;
      Hy = (-1)*Hy;
   }

   double s = (-2)*(Hx*Nx + Hy*Ny);

   Rx = s*Nx + Hx;
   Ry = s*Ny + Hy;

   speedCmd = 0.5;

   if (backwardsFlight)
   {
      Rx = (-1)*Rx;
      Ry = (-1)*Ry;
      speedCmd = -0.5;
   }
   *bounce = atan2(Ry,Rx);
}
