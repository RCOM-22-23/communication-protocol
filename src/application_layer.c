// Application layer protocol implementation

#include "applicationLayer_headers.h"
#include "application_layer.h"
#include "link_layer.h"
#include "headers.h"



LinkLayerRole getRole(const char *role){
    if(strcmp(role,"tx") == 0)
        return LlTx;
    if(strcmp(role,"rx") == 0)
        return LlRx;
        
    printf("Invalid input arguments\n");
    exit(-1);
}

LinkLayer getConnectionParams(const char *serialPort, const char *role, int baudRate,int nTries, int timeout){
    LinkLayer connectionParameters;

    LinkLayerRole appRole = getRole(role);
    connectionParameters.role = appRole;
    strcpy(connectionParameters.serialPort,serialPort);
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    return connectionParameters;
}

debugType executeLinkLayer(LinkLayer connectionParameters, Packets *packets, int packet_number){

    //<------llopen()------>
    if(llopen(connectionParameters) == 1){
        if(connectionParameters.role == LlRx)
            printf("Established Connection with transmitter\n");
        else if(connectionParameters.role == LlTx)
            printf("Established Connection with reader\n");
    }
    else{
        return(ConnectionError);
    }
    //<------llopen() end------>

    //<------llwrite()------>

    //TODO : WRITE TO SERIAL PORT FROM PACKETS STRUCT

    //<------llwrite() end------>

    //<------llread()------>

    //TODO : WRITE FROM SERIAL PORT FROM PACKETS STRUCT

    //<------llread() end------>

    //<------llclose()------>
    if(llclose(FALSE) == 1){
        if(connectionParameters.role == LlRx)
            printf("Disconnection from transmitter successful\n");
        else if(connectionParameters.role == LlTx)
            printf("Disconnection from receiver successful\n");
    }
    else{
        return(DisconnectionError);
    }
    //<------llclose() end------>


    return OK;
}

debugType readAndPackage(Packets *packets, int *packet_number, int *maxPackets,const char *filename){

    //<------Opening File BEGIN------>
    FILE *file;
    file = fopen(filename,"rb");

    if(file == NULL){
        return FileError;
    }
    //<------Opening File END------>

    //<------Reading File BEGIN------>
    size_t len;
    while((len = fread(packets[*packet_number].content,sizeof(unsigned int),PACKET_SIZE,file)) != 0){
        packets[*packet_number].size = len;
        (*packet_number)++;
        if(*(packet_number) >= *(maxPackets)){
            *(maxPackets) += 10;
            packets = (Packets *) realloc(packets, *(maxPackets));
        }
            
    }
    //<------Reading File BEGIN------>

    //<------Closing File BEGIN------>
    fclose(file);
    //<------Closing File END------>

    return OK;
}

debugType writePackage(const Packets *packets,int packet_number,const char *filename){

    if(packet_number == 0){
        return NoPacketsError;
    }

    //<------Opening File BEGIN------>
    FILE *file = fopen(filename,"wb");

    if(file == NULL){
        return FileError;
    }
    //<------Opening File END------>

    //<------Writing File BEGIN------>
    for(int i = 0; i < packet_number; i++){
        fwrite(packets[i].content,sizeof(unsigned int),packets[i].size,file);
    }
    //<------Writing File END------>

    //<------Closing File BEGIN------>
    fclose(file);
    //<------Closing File BEGIN------>

    return OK;
}


void print_packets(const Packets *packets,int packet_number){
    printf("Packet number: %d\n", packet_number);
    for(int i = 0; i < packet_number; i++){
        printf("PACKET %d SIZE : %ld\n",i,packets[i].size);
        printf("    ");
        for(int j = 0; j < packets[i].size; j++){
            printf("%X ",packets[i].content[j]);
        }
        printf("\n");
    }
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,int nTries, int timeout, const char *filename)
{
    //TODO : change to dynamic array
    int maxPackets = MAX_PACKETS;
    Packets *packets = (Packets *) malloc(sizeof(Packets)*maxPackets);
    int packet_number = 0;

    //Setting connection params from main
    LinkLayer connectionParameters = getConnectionParams(serialPort,role,baudRate,nTries,timeout);

    //Reading from file, if role is transmitter
    if(connectionParameters.role == LlTx){
        switch (readAndPackage(packets, &packet_number, &maxPackets,filename))
        {
            case FileError:
                printf("Could not read from file \"%s\", closing application\n",filename);
                exit(-1);
                break;
            default:
                printf("Read from file  \"%s\" successfully\n",filename);
                break;
        }
    }

    /*
    print_packets(packets,packet_number);
    writePackage(packets,packet_number,"output.gif");
    */

    //execute linklayer code, with error handling
    switch (executeLinkLayer(connectionParameters, packets, packet_number))
    {
        case ConnectionError:
            printf("Could not establish connection with the other program, closing application\n");
            exit(-1);
            break;
        case DisconnectionError:
            printf("Could not disconnect from the other machine, closing application\n");
            exit(-1);
            break;
        default:
            break;
    }

    //Writing to file, if role is receiver
    if(connectionParameters.role == LlRx){
        switch (writePackage(packets,packet_number,filename))
        {
            case NoPacketsError:
                printf("No packets are stored, closing application\n");
                exit(-1);
                break;
            case FileError:
                printf("Could not write to file \"%s\", closing application\n",filename);
                exit(-1);
                break;
            default:
                printf("Wrote to the file \"%s\" successfully\n",filename);
                break;
        }
    }

    free(packets);
}
