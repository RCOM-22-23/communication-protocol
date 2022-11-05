#ifndef _APPLICATION_HEADERS_H_
#define _APPLICATION_HEADERS_H_

#include <headers.h>

#define MAX_PACKETS 100

#define MAX_ATTEMPTS 3

typedef enum
{
    OK,
    ConnectionError,
    FileError,
    NoPacketsError,
    DisconnectionError,
    ReadingError,
    WritingError,
    ExceededAttempts,
} debugType;

struct Packets
{
    unsigned char content[PACKET_SIZE*2];
    size_t size;
};
typedef struct Packets Packets;

#endif // _APPLICATION_HEADERS_H_
