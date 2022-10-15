#ifndef _APPLICATION_HEADERS_H_
#define _APPLICATION_HEADERS_H_

#include <stdio.h>
#define PACKET_SIZE 256
#define MAX_PACKETS 100

typedef enum
{
    OK,
    ConnectionError,
    FileError,
    NoPacketsError,
    DisconnectionError,
} debugType;

struct Packets
{
    unsigned int content[PACKET_SIZE];
    size_t size;
};
typedef struct Packets Packets;

#endif // _APPLICATION_HEADERS_H_
