#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMMenus.h"
#include "XPLMUtilities.h"
#include "XPLMGraphics.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"
#include "XPLMProcessing.h"
#include <functional>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <pthread.h>
#include <signal.h>

#if IBM
	#include <windows.h>
#endif
#ifndef XPLM300
	#error This is made to be compiled against the XPLM300 SDK
#endif

int fd = -1;

static XPLMDataRef  gCOM1StandbyMhz;
static XPLMDataRef  gCOM1Standbykhz;
static XPLMDataRef  gCOM1ActiveMhz;
static XPLMDataRef  gCOM1Activekhz;

static XPLMDataRef  gNAV1StandbyMhz;
static XPLMDataRef  gNAV1Standbykhz;
static XPLMDataRef  gNAV1ActiveMhz;
static XPLMDataRef  gNAV1Activekhz;


#define COM_MODE 3
#define NAV_MODE 4
int navActive = 0;
int navStandby = 0;
int comActive = 0;
int comStandby = 0;
char currentMode = 0;
char needsUpdate = 1;
pthread_mutex_t lock;
pthread_t thread_id;

int set_interface_attribs (int fd, int speed, int parity){
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0){
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0){
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block){
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0){
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0){}
}

void* checkRadio(void* args){
  char data[50];
  while(1){
    data[0] = '\0';
    int read_i = 0; 
    char c;
    while(read_i<50 && read(fd, &c, 1) > 0){
      data[read_i] = c;
      read_i++;
      if(c == '\n' || c == '\r'){
        data[read_i] = '\0';
        read_i = 0;
        uint32_t active;
        uint32_t standby;
        uint8_t mode;
        if(3 == sscanf(data, "AVRADIO A:%6d S:%6d M:%1hhd", &active, &standby, &mode)){
          pthread_mutex_lock(&lock);
          if(mode == COM_MODE){
            comActive = active;
            comStandby = standby;
            currentMode = COM_MODE;
            needsUpdate = 1;
          }
          if(mode == NAV_MODE){
            navActive = active;
            navStandby = standby;
            currentMode = NAV_MODE;
            needsUpdate = 1;
          }
          pthread_mutex_unlock(&lock);
        }
      }
    }
    usleep(50000);
  }
}


float	CheckRadioCallback(
                                   float                inElapsedSinceLastCall,    
                                   float                inElapsedTimeSinceLastFlightLoop,    
                                   int                  inCounter,    
                                   void *               inRefcon)
{

  pthread_mutex_lock(&lock);
  if(needsUpdate){
    if(currentMode == NAV_MODE){
        XPLMSetDatai(gNAV1ActiveMhz, navActive/1000);
        XPLMSetDatai(gNAV1Activekhz, (navActive - ((navActive/1000)*1000))/10);
        
        XPLMSetDatai(gNAV1StandbyMhz, navStandby/1000);
        XPLMSetDatai(gNAV1Standbykhz, (navStandby - ((navStandby/1000)*1000))/10);
    }
    if(currentMode == COM_MODE){
        XPLMSetDatai(gCOM1ActiveMhz, comActive/1000);
        XPLMSetDatai(gCOM1Activekhz, comActive - ((comActive/1000)*1000));
        
        XPLMSetDatai(gCOM1StandbyMhz, comStandby/1000);
        XPLMSetDatai(gCOM1Standbykhz, comStandby - ((comStandby/1000)*1000));
    }
    needsUpdate = 0; 
  }
  pthread_mutex_unlock(&lock);
  return 1.0;
}


PLUGIN_API int XPluginStart(
					 char *        outName,
					 char *        outSig,
					 char *        outDesc)
{
	// Plugin Info
	strcpy(outName, "aviradio");
	strcpy(outSig, "warpenguin.aviradio");
	strcpy(outDesc, "External radio peripheral");

	char portname[] = "/dev/ttyACM0";
	
	int i = 0;
	while(fd < 0){
		portname[11] = '0' + i;
		fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
		i++;
		if(i >= 10)
			i = 0;
		usleep(200000);
	}
  set_interface_attribs (fd, B9600, 1);  // set speed to 115,200 bps, 8n1 (no parity)
	//set_interface_attribs (fd, B115200, 1);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking

  gCOM1StandbyMhz = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_standby_frequency_Mhz");
  gCOM1Standbykhz = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_standby_frequency_khz");
  gCOM1ActiveMhz = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_frequency_Mhz");
  gCOM1Activekhz = XPLMFindDataRef("sim/cockpit2/radios/actuators/com1_frequency_khz");

  gNAV1StandbyMhz = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_standby_frequency_Mhz");
  gNAV1Standbykhz = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_standby_frequency_khz");
  gNAV1ActiveMhz = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_frequency_Mhz");
  gNAV1Activekhz = XPLMFindDataRef("sim/cockpit2/radios/actuators/nav1_frequency_khz");

  pthread_mutex_init(&lock, NULL);

  pthread_create(&thread_id, NULL, &checkRadio, NULL);

  XPLMRegisterFlightLoopCallback(		
			CheckRadioCallback,	/* Callback */
			0.5,					/* Interval */
			NULL);					/* refcon not used. */

	return 1;
}

PLUGIN_API void    XPluginStop(void)
{
  pthread_kill(thread_id, 9);
  pthread_join(thread_id, NULL);
}


PLUGIN_API void XPluginDisable(void){}

PLUGIN_API int XPluginEnable(void){	return 1;}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho,long inMessage, void *inParam){}

