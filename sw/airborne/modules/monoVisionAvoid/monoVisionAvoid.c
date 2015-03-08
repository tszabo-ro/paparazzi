#include "monoVisionAvoid.h"
#include "../../firmwares/rotorcraft/navigation.h"          // nav_heading is defined here!
#include "../../math/pprz_geodetic_float.h"                 // struct EnuCoor_f is defined here
#include "../../math/pprz_geodetic_int.h"                   // struct EnuCoor_i is defined here
#include "state.h"                                          // for stateGetPosition_f()


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
#include <time.h>

/*
The packet struct to be received by this module is defined as follows:
NOTE: All data in the packet shall be of type: uint16_t

Position    Data
0           0xFFFF
1           0xFFFF
2           Message Index:- A simple counter from 1 upwards. Eventually will OVERFLOW, so do the counting in the appropriate datatype: used to ensure that always the newest message is applied.
3           Turn Rate:    - Turn rate effort of the drone: 0= max turn rate LEFT, max(uint16_t) = max turn rate RIGHT.
4           Velocity cmd: - Commanded forward velocity effort: 0=0 velocity, max(uint16_t) = max fwd velocity
5           Checksum:     - Used to check packet validity. Calculate as: sum the values from positions 2,3 & 4 up in a uint16_t and apply a BITWISE NOT operation to it.

The "effort" commands are the percentages of the maximum turn rate & velocity values defined in monoVisionAvoid.h

Example (in C, but you'll get the point):

uint16_t msgCounter = 0;  // <- Don't get me wrong, I don't think you're an idiot, but this should be somewhere OUTSIDE of this function. I wrote it here for the sake of declaring it as uint16_t!

uint16_t transmitBuffer[6];
transmitBuffer[0] = 0xFFFF;
transmitBuffer[1] = 0xFFFF;
transmitBuffer[2] = msgCounter++
transmitBuffer[3] = 12345;  // <- This is the turn rate effort as defined above
transmitBuffer[4] = 0;  // <- This is the velocity effort as defined above
transmitBuffer[5] = ~(transmitBuffer[2] + transmitBuffer[3] + transmitBuffer[4]);

sendto(droneSocket, (void*)&transmitBuffer, 12); <- In total there are 12 bytes in the packet!
*/


int                 sock;
int                 bytes_recv;
int                 sin_size;
struct sockaddr_in  server_addr;
struct hostent      *host;
uint8_t             recv_buffer[12];

unsigned long long  lastMsgRecvTime;
uint16_t            lastMsgIndex;

struct EnuCoor_i    newTargetLocation;
int32_t             newHeadingSetpoint;

void monoVisionAvoid_init(void)
{
  host = (struct hostent *) gethostbyname((char *)"192.168.1.1"); // NOTE: This should be the IP address of the interface where the packets are coming from!
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("socket");
    exit(1);
  }

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(3456);
  server_addr.sin_addr = *((struct in_addr *)host->h_addr);
  bzero(&(server_addr.sin_zero), 8);
  sin_size = sizeof(struct sockaddr);

  if (bind(sock, (struct sockaddr *)&server_addr,
           sizeof(struct sockaddr)) == -1) {
    perror("Bind");
    exit(1);
  }
}
void monoVisionAvoid_periodic(void)
{

  float psiDotCmd      = 0;
  float vCmd           = 0;
  
  struct timeval tv;
  gettimeofday(&tv,NULL);
  unsigned long long cTime = tv.tv_sec*1000000 + tv.tv_usec;
  
  
  bytes_recv = recvfrom(sock, recv_buffer, 12, MSG_DONTWAIT, (struct sockaddr *)&server_addr, (socklen_t *)&sin_size);
  
  // Check packet validity!
  if ( (bytes_recv >= 12) &&
      ( 
        (recv_buffer[0] == 0xFFFF) && 
        (recv_buffer[1] == 0xFFFF) && 
        (recv_buffer[5] == ~(recv_buffer[2] + recv_buffer[3] + recv_buffer[4]) )
      ) && 
      ( 
        (recv_buffer[2] > lastMsgIndex) || 
        (
          (recv_buffer[2] == 0) && 
          (lastMsgIndex == UINT16_MAX)
        ) 
      )
    )
  {
    lastMsgRecvTime = cTime;
    psiDotCmd       = (float)((((double)recv_buffer[3])/((double)UINT16_MAX))*2 - 1);
    vCmd            = (float)(((double)recv_buffer[4])/((double)UINT16_MAX));
  }
  
  if ( (cTime - lastMsgRecvTime) > 1000000 )  // The last packet was received more than 1 second ago. Stop moving & return
  { 
    return;
  }
  
  float newX        = stateGetPositionEnu_f()->x;
  float newY        = stateGetPositionEnu_f()->y;
  float cmdHeading  = ANGLE_FLOAT_OF_BFP(nav_heading);
  
  cmdHeading        += psiDotCmd*MODULE_UPDATE_RATE;    // This is in NED coordinates.
  
  // since cmdHeading is in NED, the sin/cos of the axis are switched in ENU
  newX              += vCmd*sin(cmdHeading);
  newX              += vCmd*cos(cmdHeading);
  
  newTargetLocation.x = POS_BFP_OF_REAL(newX);
  newTargetLocation.y = POS_BFP_OF_REAL(newY);
  newTargetLocation.z = stateGetPositionEnu_i()->z;
  
  newHeadingSetpoint  = ANGLE_BFP_OF_REAL(cmdHeading);
}
bool monoVisionAvoid_flightPlanUpdate(void)
{
  nav_heading         = newHeadingSetpoint;
  navigation_target.x = newTargetLocation.x;
  navigation_target.y = newTargetLocation.y;
  navigation_target.z = newTargetLocation.z;
  
  return 0;
}
