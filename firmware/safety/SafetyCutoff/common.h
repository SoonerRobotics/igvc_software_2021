#ifndef COMMON_H
#define COMMON_H

#define GLOBAL_PASSWORD "CRABCAKES"

struct RadioPacket {
    unsigned char   id;
    char            password[sizeof(GLOBAL_PASSWORD)];
    char            message[16];
};

#endif