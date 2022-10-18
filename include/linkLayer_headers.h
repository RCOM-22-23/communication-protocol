#ifndef _LINK_HEADERS_H_
#define _LINK_HEADERS_H_


#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

//<----------SERIAL PORT CONFIG---------->

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

volatile int STOP = FALSE;

//<----------SERIAL PORT CONFIG END---------->


//<----------FRAMES---------->

#define CONTROL_FRAME_SIZE 5

//END & START FLAG
#define F 0x7E

//ESCAPE
#define ESC 0x7D

//ADDRESS FIELD
// --A_T is for commands sent by the transmitter and responses sent by the Reader.
#define A_T 0x03
// --A_R is for commands sent by the Reader and responses sent by the transmitter.
#define A_R 0x01

//CONTROL FIELD
#define SET 0x03
#define DISC 0x0B
#define UA 0x07

#define BCC1_SET A_T^SET
#define BCC1_UA_R A_R^UA
#define BCC1_UA_T A_T^UA

#define BCC1_DISC_T A_T^DISC
#define BCC1_DISC_R A_R^DISC

// I FRAMES
#define CONTROLO_0 0x00
#define CONTROLO_1 0x40

//RESPONDES TO I FRAMES

#define RR_CONTROLO_0 0x05
#define RR_CONTROLO_1 0x85
#define REJ_CONTROLO_0 0x01
#define REJ_CONTROLO_1 0x81


//<----------FRAMES END---------->

//<----------STATES---------->
#define START 1
#define FLAG_RCV 2
#define A_RCV 3
#define C_RCV 4
#define BCC_OK 5
#define STOP 6

//<----------OTHER---------->

#define BUF_SIZE 256

//<----------OTHER END---------->


//<----------GLOBAL VARIABLES---------->

//SET format
unsigned char set[CONTROL_FRAME_SIZE] = {F,A_T,SET,BCC1_SET,F};

//UA format
unsigned char ua_R[CONTROL_FRAME_SIZE] = {F,A_R,UA,BCC1_UA_R,F};

unsigned char ua_T[CONTROL_FRAME_SIZE] = {F,A_T,UA,BCC1_UA_T,F};

//DISC format
unsigned char disc_T[CONTROL_FRAME_SIZE] = {F,A_T,DISC,BCC1_DISC_T,F};

unsigned char disc_R[CONTROL_FRAME_SIZE] = {F,A_R,DISC,BCC1_DISC_R,F};


//global serial port descriptor
int fd;

//Alarm and timeout variables
int alarmEnabled = FALSE;
int alarmCount = 0;
int ua_R_received = FALSE;
int ua_T_received = FALSE;
int set_received = FALSE;
int disc_received_R = FALSE;
int disc_received_T = FALSE;



struct termios oldtio;
struct termios newtio;

int attempts;
int timeout;
LinkLayerRole role;
//<----------GLOBAL VARIABLES END---------->

#endif // _LINK_HEADERS_H_
