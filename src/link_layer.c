// Link layer protocol implementation

#include "link_layer.h"
#include "headers.h"
#include "linkLayer_headers.h"


// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

void openSerialPort(LinkLayer connectionParameters){
    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");
}

int check_state(unsigned char read_char,unsigned char wanted_char, int new_state, int *current_state){
    if(read_char == wanted_char){
        *current_state = new_state;
        return TRUE;
    }
    return FALSE;
}

void attempt_readSet(int signal){
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Connection Failed, retrying (%d/%d)\n",alarmCount,attempts);
}

//State machine for reading set messages.
int read_SET(LinkLayer connectionParameters){
    //small buffer for reading from serial port
    unsigned char buf[2];
    // Set alarm function handler
    (void)signal(SIGALRM, attempt_readSet);

    int state = START;
    while(state != STOP && alarmCount < connectionParameters.nRetransmissions){
        int bytes = read(fd, buf, 1);
        unsigned char read_char = buf[0];
        if (alarmEnabled == FALSE) {
                alarm(connectionParameters.timeout); // Set alarm to be triggered in <connectionParameters.timeout> seconds
                alarmEnabled = TRUE;
        } 
        if(bytes != 0){
            switch(state){
                case START:
                    if(!check_state(read_char,F,FLAG_RCV,&state))
                        state = START;
                    break;
                case FLAG_RCV:
                    if(!(check_state(read_char,A_W,A_RCV,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case A_RCV:
                    if(!(check_state(read_char,SET,C_RCV,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case C_RCV:
                    if(!(check_state(read_char,BCC1_SET,BCC_OK,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case BCC_OK:
                    if(!check_state(read_char,F,STOP,&state))
                        state = START;
                    else{
                        set_received = TRUE;
                    }
                    break;
                default:
                    break;
            }
        }      
    }

    return set_received;
}

void send_UA(){
    write(fd, ua, CONTROL_FRAME_SIZE);
    printf("Sent UA to transmitter\n");
}


void send_SET(){
    write(fd, set, CONTROL_FRAME_SIZE);
    printf("SET sent to receiver \n");
}

int receive_UA(){
    //small buffer for reading from serial port
    unsigned char buf[2];

    int state = START;
    while(state != STOP){
        int bytes = read(fd, buf, 1);
        unsigned char read_char = buf[0];
        if(bytes != 0){
            switch(state){
                case START:
                    if(!check_state(read_char,F,FLAG_RCV,&state))
                        state = START;
                    break;
                case FLAG_RCV:
                    if(!(check_state(read_char,A_R,A_RCV,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case A_RCV:
                    if(!(check_state(read_char,UA,C_RCV,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case C_RCV:
                    if(!(check_state(read_char,BCC1_UA,BCC_OK,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case BCC_OK:
                    if(!check_state(read_char,F,STOP,&state))
                        state = START;
                    break;
                default:
                    break;
            }
        }
        else{
            return FALSE;
        } 
    }
    return TRUE;
}

// Alarm function handler
void connectionAttempt(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    send_SET();

    // Wait until all bytes have been written to the serial port
    // TODO : Keep or remove ? 
    sleep(1);

    if(receive_UA() == TRUE){
        ua_received = TRUE;
    }
    else{
        printf("Connection Failed, retrying in %d seconds (%d/%d)\n",timeout,alarmCount,attempts);
    }
}


int llopen_transmitter(LinkLayer connectionParameters){
    timeout = connectionParameters.timeout;
    // Set alarm function handler
    (void)signal(SIGALRM, connectionAttempt);

    while (alarmCount < attempts && ua_received == FALSE)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(connectionParameters.timeout); // Set alarm to be triggered in <connectionParameters.timeout> seconds
            alarmEnabled = TRUE;
        }
    }

    if(ua_received == TRUE){
        alarmEnabled = FALSE;
        alarmCount = 0;
        return 1;
    }
    else{
        return -1;
    }
}

int llopen_reader(LinkLayer connectionParameters){
    if(read_SET(connectionParameters) == TRUE){
        send_UA();
        return 1; 
    }
    
    return -1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////

int llopen(LinkLayer connectionParameters){
    openSerialPort(connectionParameters);
    attempts = connectionParameters.nRetransmissions;


    if(connectionParameters.role == LlRx){
        return llopen_reader(connectionParameters);
    }

    else{
        if(connectionParameters.role == LlTx){
            return llopen_transmitter(connectionParameters);
        }
    }
       
    return -1;
}


//Byte Stuffing of the frame I
unsigned char *byte_stufffing(unsigned char *frame, unsigned int *length){
    unsigned char *stuffed_frame = (unsigned char*) malloc((*length)*sizeof(char));
    
    unsigned int Length = *length;

    stuffed_frame[0] = F;

    int j = 1;

    for(int i = 0; i < *length; i++){
        if(frame[i] == F){
            Length++;
            stuffed_frame = (unsigned char *) realloc(stuffed_frame, Length);
            stuffed_frame[j] = ESC;
            stuffed_frame[++j] = 0x5E;
            j++;
            continue;    
        }
        else if(frame[i] == ESC){
            Length++;
            stuffed_frame = (unsigned char*) realloc(stuffed_frame, Length);
            stuffed_frame[j] = ESC;
            stuffed_frame[++j] = 0x5D;
            j++;
            continue;
        }
        else{
            stuffed_frame[j] = frame[i];
            j++;
        }
    }
    
    stuffed_frame[j] = F;
    *length = Length;


   return stuffed_frame;
}

//Destuffing of the frame I
unsigned char *byte_Destuffing(unsigned char *stuffed_frame, unsigned int *length){

    unsigned int Length = 0;
    unsigned char *destuffed_frame = malloc(Length);

    for(int i = 0; i < *length; i++){
        if(stuffed_frame[i] == ESC && stuffed_frame[i+1] == 0x5E){
           Length++; 
           destuffed_frame = (unsigned char *) realloc(destuffed_frame, Length);
           destuffed_frame[Length-1] = F;
           i++;
           continue;
        }

        else if(stuffed_frame[i] == ESC && stuffed_frame[i+1] == 0x5D){
           Length++; 
           destuffed_frame = (unsigned char *) realloc(destuffed_frame, Length);
           destuffed_frame[Length-1] = ESC;
           i++;
           continue;
        }
        else{
           Length++; 
           destuffed_frame = (unsigned char *) realloc(destuffed_frame, Length);
           destuffed_frame[Length-1] = stuffed_frame[i];

        }
    }
   
   *length = Length;

  
  return destuffed_frame;
}


////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize){
   

    
       
 
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet){
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1){
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 1;
}
