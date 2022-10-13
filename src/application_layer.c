// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include "headers.h"
#include "applicationLayer_headers.h"

typedef enum
{
    OK,
    ConnectionError,
} debugType;


LinkLayerRole getRole(const char *role){
    if(strcmp(role,"tx") == 0)
        return LlTx;
    if(strcmp(role,"rx") == 0)
        return LlRx;
        
    printf("Invalid input arguments\n");
    exit(-1);
}

LinkLayer getConnectionParams(const char *serialPort, const char *role, int baudRate,int nTries, int timeout, const char *filename){
    LinkLayer connectionParameters;

    LinkLayerRole appRole = getRole(role);
    connectionParameters.role = appRole;
    strcpy(connectionParameters.serialPort,serialPort);
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    return connectionParameters;
}

debugType executeLinkLayer(LinkLayer connectionParamenters){

    //<------llopen()------>
    if(llopen(connectionParamenters) == 1){
        if(connectionParamenters.role == LlRx)
            printf("Established Connection with transmitter\n");
        else if(connectionParamenters.role == LlTx)
            printf("Established Connection with reader\n");
    }
    else{
        return(ConnectionError);
    }
    //<------llopen() end------>

    //<------llopen()------>
        llclose(0);
    //<------llopen() close------>


    return OK;
}


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    //Setting connection params from main
    LinkLayer connectionParameters = getConnectionParams(serialPort,role,baudRate,nTries,timeout,filename);

    //execute linklayer code, with error handling
    switch (executeLinkLayer(connectionParameters))
    {
    case ConnectionError:
        printf("Could not establish connection with the other program, closing application\n");
        exit(-1);
        break;
    default:
        break;
    }
}
