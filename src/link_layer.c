// Link layer protocol implementation

#include "link_layer.h"
#include "headers.h"
#include "linkLayer_headers.h"


// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source


int check_state(unsigned char read_char,unsigned char wanted_char, int new_state, int *current_state){
    if(read_char == wanted_char){
        *current_state = new_state;
        return TRUE;
    }
    return FALSE;
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

void alarm_read(int signal){
    alarmEnabled = FALSE;
    alarmCount++;
}


int read_control(unsigned char control_byte,LinkLayerRole role){
    unsigned char role_byte;
    if(role == LlRx) role_byte = A_T;
    if(role == LlTx) role_byte = A_R;

    //small buffer for reading from serial port
    unsigned char buf[2];
    alarmEnabled = TRUE;

    (void)signal(SIGALRM, alarm_read);
    alarm(timeout);

    int state = START;
    while(state != STOP){
        if(alarmEnabled == FALSE)
            return FALSE;
        int bytes = read(fd, buf, 1);
        unsigned char read_char = buf[0];
        if(bytes != 0){
            switch(state){
                case START:
                    if(!check_state(read_char,F,FLAG_RCV,&state))
                        state = START;
                    break;
                case FLAG_RCV:
                    if(!(check_state(read_char,role_byte,A_RCV,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case A_RCV:
                    if(!(check_state(read_char,control_byte,C_RCV,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case C_RCV:
                    if(!(check_state(read_char,role_byte^control_byte,BCC_OK,&state) || check_state(read_char,F,FLAG_RCV,&state)))
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
    }
    return TRUE;

}


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





void send_UA_R(){
    write(fd, ua_R, CONTROL_FRAME_SIZE);
    printf("Sent UA to transmitter\n");
}

void send_UA_T(){
    write(fd, ua_T, CONTROL_FRAME_SIZE);
    printf("Sent UA to receiver\n");
}

void send_SET(){
    write(fd, set, CONTROL_FRAME_SIZE);
    printf("SET sent to receiver \n");
}

// Alarm function handler
void connectionAttempt()
{
    send_SET();

    if(read_control(UA,LlTx) == TRUE){
        ua_R_received = TRUE;
    }
    else{
        printf("Connection Failed, retrying in %d seconds (%d/%d)\n",timeout,alarmCount,attempts);
    }
}


int llopen_transmitter(LinkLayer connectionParameters){
    printf("---------Attempting to establish connection with reader---------\n");

    timeout = connectionParameters.timeout;

    while (alarmCount < attempts && ua_R_received == FALSE)
    {
        connectionAttempt();
    }

    if(ua_R_received == TRUE){
        alarmEnabled = FALSE;
        alarmCount = 0;
        return 1;
    }
    else{
        return -1;
    }
}

int llopen_reader(LinkLayer connectionParameters){
    printf("---------Attempting to establish connection with transmitter---------\n");

    timeout = connectionParameters.timeout;

    while (alarmCount < attempts && set_T_received == FALSE)
    {
        if(read_control(SET,LlRx) == TRUE){
            set_T_received = TRUE;
            send_UA_R();
            return 1; 
        }
        else{
            printf("Connection Failed, retrying in %d seconds (%d/%d)\n",timeout,alarmCount,attempts);
        }
    }
  
    return -1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////

int llopen(LinkLayer connectionParameters){
    openSerialPort(connectionParameters);
    attempts = connectionParameters.nRetransmissions;
    role = connectionParameters.role;

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
////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int send_I_frame(const unsigned char *buf, int bufSize){
    unsigned int size_frameI = bufSize + 6;
    unsigned char frameI[size_frameI];
    unsigned char bcc2 = 0x00;

    //Setting Flag
    frameI[0] = F;
    //Setting A
    frameI[1] = A_T;
    
    //Setting C
    if(number_seq == 0){
        frameI[2] = I_0;
    }
    else if(number_seq == 1){
        frameI[2] = I_1;
    }

    //Setting BCC1
    frameI[3] = frameI[1] ^ frameI[2];

    //Setting D1-DN
    for(int i = 0; i < bufSize; i++){
        frameI[i+4] = buf[i];
        bcc2 = bcc2 ^ buf[i];
    }
    
    //Setting BCC2
    frameI[size_frameI - 2] = bcc2;
    //Setting FLAG
    frameI[size_frameI - 1] = F;

    //TODO : Change this -> Stuffing should only be done to D1-DN
    //unsigned char *stuffed = byte_stufffing(frameI,&size_frameI);
    
    int bytes = write(fd,frameI,size_frameI);
    return bytes;
}

//read_RR returns 1 if it reads RR_1, 0 if it reads RR_0, and -2 if reads REJ_0, -3 for REJ_1 and -1 for nothing at all.
//TODO FIX READ_RR()
int read_RR(){
    unsigned char role_byte = A_R;
    int return_value = -1;

    //small buffer for reading from serial port
    unsigned char buf[2];
    alarmEnabled = TRUE;

    (void)signal(SIGALRM, alarm_read);
    alarm(timeout);

    int sequence_number = -1;

    int state = START;
    while(state != STOP){
        if(alarmEnabled == FALSE)
            return FALSE;
        int bytes = read(fd, buf, 1);
        unsigned char read_char = buf[0];
        if(bytes != 0){
            switch(state){
                case START:
                    sequence_number = -1;
                    if(!check_state(read_char,F,FLAG_RCV,&state))
                        state = START;
                    break;
                case FLAG_RCV:
                    if(!(check_state(read_char,role_byte,A_RCV,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case A_RCV:
                    if(check_state(read_char,RR_0,RR_0_RCV,&state)){
                        sequence_number = 0;
                    }
                    else if(check_state(read_char,RR_1,RR_1_RCV,&state)){
                        sequence_number = 1;
                    }
                    else if(check_state(read_char,REJ_0,REJ_0_RCV,&state)){
                        sequence_number = 1;
                    }
                    else if(check_state(read_char,REJ_1,REJ_1_RCV,&state)){
                        sequence_number = 0;
                    }
                    if(!(sequence_number != -1 || check_state(read_char,REJ_0,REJ_RCV,&state) || check_state(read_char,REJ_1,REJ_RCV,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case RR_0_RCV:
                    if(!(check_state(read_char,role_byte^RR_0,BCC_OK,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case RR_1_RCV:
                    if(!(check_state(read_char,role_byte^RR_1,BCC_OK,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case REJ_0_RCV:
                    if(!(check_state(read_char,role_byte^REJ_0,BCC_OK,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case REJ_1_RCV:
                    if(!(check_state(read_char,role_byte^REJ_1,BCC_OK,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case BCC_OK:
                    if(!check_state(read_char,F,STOP,&state)){
                        state = START;
                    }
                    else{
                        return_value = sequence_number;
                    }
                    break;
                default:
                    break;
            }
        }
    }
    return return_value;
}

//Returns 0 on retrying sending, 1 on success, -1 on failure (which closes the program)
int llwrite(const unsigned char *buf, int bufSize){
    int rr_value;
    int rr_received = FALSE;
    int rej_received = FALSE;
    alarmCount = 0;

    send_I_frame(buf,bufSize);
    printf("Sent I(%d) frame to receiver\n",number_seq);

    int prev_number_seq = number_seq;
    while (alarmCount < attempts && rr_received == FALSE && rej_received == FALSE)
    {
        rr_value = read_RR();
        if(rr_value == 1 || rr_value == 0){
            printf("Received RR(%d) from receiver\n",rr_value);
            number_seq = rr_value;
            rr_received = TRUE;
        }
        else if(rr_value == -2 || rr_value == -3){
            int print_value = 0;
            if(rr_value == -3) print_value = 1;
            printf("Received REJ(%d) from receiver\n",print_value);
            number_seq = rr_value;
            rej_received = TRUE;
        }
        else{
            printf("Could not receive RR or REJ, retrying in %d seconds (%d/%d)\n",timeout,alarmCount,attempts);
        }
    }
    if(rr_received == TRUE || rej_received == TRUE){
        if (prev_number_seq == number_seq) return 0;
        return 1;
    }
    return -1;
}
    


////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////


//read_I returns 1 if it reads I_1, 0 if it reads I_0, 2 if it reads DISC, and -1 if reads nothing at all
int read_I(unsigned char *packet){
    unsigned char role_byte = A_T;
    int return_value = -1;
    packet_counter = 0;

    //small buffer for reading from serial port
    unsigned char buf[2];
    alarmEnabled = TRUE;

    (void)signal(SIGALRM, alarm_read);
    alarm(timeout);
    
    int sequence_number = -1;

    int state = START;
    while(state != STOP){
        if(alarmEnabled == FALSE)
            return FALSE;
        int bytes = read(fd, buf, 1);
        unsigned char read_char = buf[0];
        if(bytes != 0){
            switch(state){
                case START:
                    sequence_number = -1;
                    if(!check_state(read_char,F,FLAG_RCV,&state))
                        state = START;
                    break;
                case FLAG_RCV:
                    if(!(check_state(read_char,role_byte,A_RCV,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case A_RCV:
                    if(check_state(read_char,I_0,I_0_RCV,&state)){
                        sequence_number = 0;
                    }
                    else if(check_state(read_char,I_1,I_1_RCV,&state)){
                        sequence_number = 1;
                    }
                    else if(check_state(read_char,DISC,DISC_RCV,&state)){
                        sequence_number = 2;
                    }
                    if(!(sequence_number != -1 || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case I_0_RCV:
                    if(!(check_state(read_char,role_byte^I_0,BCC_I_OK,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case I_1_RCV:
                    if(!(check_state(read_char,role_byte^I_1,BCC_I_OK,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case DISC_RCV:
                    if(!(check_state(read_char,role_byte^DISC,BCC_DISC_OK,&state) || check_state(read_char,F,FLAG_RCV,&state)))
                        state = START;
                    break;
                case BCC_DISC_OK:
                    if(!check_state(read_char,F,STOP,&state)){
                        state = START;
                    }
                    else{
                        return_value = sequence_number;
                    }
                    break;
                case BCC_I_OK:
                    if(!check_state(read_char,F,STOP,&state)){
                        //TODO : destuffing
                        packet[packet_counter] = read_char;
                        packet_counter++;
                    }
                    break;
                default:
                    break;
            }
        }
    }
    return return_value;
}

void switch_expected_packet(){
    if (expected_packet == 1){
        expected_packet = 0;
    }
    else if(expected_packet == 0){
        expected_packet = 1;
    }
}

void send_RR(){
    if(expected_packet == 0)
        write(fd, rr0, CONTROL_FRAME_SIZE);
    if(expected_packet == 1)
        write(fd, rr1, CONTROL_FRAME_SIZE);

    printf("RR(%d) sent to transmitter\n",expected_packet);
}


void send_REJ(){
     if(expected_packet == 0)
        write(fd, rej1, CONTROL_FRAME_SIZE);
    if(expected_packet == 1)
        write(fd, rej0, CONTROL_FRAME_SIZE);

    printf("REJ sent to transmitter\n");
}

void checkBCC2(unsigned char *packet,int *I_value){
    unsigned char bcc2 = 0x00;
    for(int i = 0; i < packet_counter-1; i++){
        bcc2 = bcc2^packet[i];
    }
    if(bcc2 != packet[packet_counter-1]){
        *I_value = -1;
        printf("Invalid BCC2\n");  
    }
        

}

//Returns 0 on retrying reading, 1 on success, -1 on failure (which closes the program), 2 on disc
int llread(unsigned char *packet){
    int I_value;
    int I_received = FALSE;
    alarmCount = 0;
    packet_counter = 0;

    while (alarmCount < attempts && I_received == FALSE)
    {
        I_value = read_I(packet);
        //checkBCC2(packet,&I_value);
        if(I_value != -1){
            number_seq = I_value;
            I_received = TRUE;
        }
        else{
            printf("Could not receive I or DISC, retrying in %d seconds (%d/%d)\n",timeout,alarmCount,attempts);
        }
    }
    if(I_received == TRUE){
        //Disc received
        if(number_seq == 2){
            printf("Read DISC from Transmitter\n");
            return 2;
        }
            
        //got the expected packet
        if (expected_packet == number_seq){
            printf("Received I(%d) from the transmitter\n",number_seq);
            switch_expected_packet();
            send_RR();
            return 1;
        } 

        //did not receive expected packet
        send_REJ();
        return 0;
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

void close_SerialPort(){
    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1){
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
}

void send_DISC(){
    if(role == LlTx){
        write(fd, disc_T, CONTROL_FRAME_SIZE);
        printf("DISC sent to receiver\n");
    }
    else if(role == LlRx){
        write(fd, disc_R, CONTROL_FRAME_SIZE);
        printf("DISC sent to transmitter\n");
    }   
}

void disconnectionAttempt_T(){
    if(read_control(DISC,LlTx) == TRUE){
        disc_received_R = TRUE;
    }
    else{
        printf("Could not read DISC, retrying in %d seconds (%d/%d)\n",timeout,alarmCount,attempts);
    }
}

void disconnectionAttempt_R_1(){
    if(read_control(DISC,LlRx) == TRUE){
        disc_received_T = TRUE;
    }
    else{
        printf("Could not read DISC, retrying in %d seconds (%d/%d)\n",timeout,alarmCount,attempts);
    }
}

void disconnectionAttempt_R_2(){
    if(read_control(UA,LlRx) == TRUE){
        ua_T_received = TRUE;
    }
    else{
        printf("Could not read UA, retrying in %d seconds (%d/%d)\n",timeout,alarmCount,attempts);
    }
}

int llclose(int showStatistics)
{
    alarmEnabled = FALSE;
    alarmCount = 0;

    //Transmitter
    if(role == LlTx){
        printf("---------Disconnecting from receiver---------\n");

        send_DISC();
        while (alarmCount < attempts && disc_received_R == FALSE)
        {
            //Send and wait for DISC from other program
            disconnectionAttempt_T();
        }
        if(disc_received_R == TRUE){
            //Received disc from transmitter
            send_UA_T();
            close_SerialPort();
            return 1;
        }
        else{
            //Did not receive DISC from transmitter
            return -1;
        }
    }
    //Receiver
    else if(role == LlRx){
        printf("---------Disconnecting from transmitter---------\n");

        printf("Received DISC from transmitter\n");
        alarmEnabled = FALSE;
        alarmCount = 0;
        send_DISC();

        //Read UA
        while (alarmCount < attempts && ua_T_received == FALSE)
        {
            //Wait for UA from other program
            disconnectionAttempt_R_2();
        }
        if(ua_T_received == TRUE){
            printf("Received UA from transmitter\n");
            close_SerialPort();
                return 1;
        }
    }
        
    return -1;
}
