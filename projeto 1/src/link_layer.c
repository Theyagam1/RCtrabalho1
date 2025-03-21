// Link layer protocol implementation

#include "link_layer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

typedef enum { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP } State;

#define FLAG 0x7E
#define A_SENDER 0x03
#define A_RECEIVER 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define C_RR0 0x05
#define C_RR1 0x85
#define C_REJ0 0x01
#define C_REJ1 0x81
#define C_I0 0x00
#define C_I1 0x40
#define ESC 0x7D
#define STUFF 0x20
#define MAX_PAYLOAD 1000


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int alarmFlag = 0;
int nrtries;
int timeout;

void byteStuffing(unsigned char *input, int *inputsize, unsigned char *output, int *outputsize) {
    *outputsize = 0;
    for (int i = 0; i < *inputsize; i++) {
        if (input[i] == FLAG || input[i] == ESC) {
            output[(*outputsize)++] = ESC;
            output[(*outputsize)++] = input[i] ^ STUFF;
        } else {
            output[(*outputsize)++] = input[i];
        }
    }
}
void byteDeStuffing(unsigned char *input, int *inputsize, unsigned char *output, int *outputsize) {
    *outputsize = 0;
    for (int i = 0; i < inputsize; i++) {
        if (input[i] == ESC) {
            i++;
            if (input[i] == FLAG^STUFF) {
                output[(*outputsize)++] = FLAG;
            } 
            else if (input[i] == ESC^STUFF) {
                output[(*outputsize)++] = ESC;
            }
            output[(*outputsize)++] = input[i];
        }
        else {
            output[(*outputsize)++] = input[i];
        }
    }
}

void alarmHandler(int signo) {
    alarmFlag = 1;
}


int llopen(LinkLayer connectionParameters) {
    int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror(connectionParameters.serialPort);
        return -1;
    }

    struct termios oldtio, newtio;
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 1;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    signal(SIGALRM, alarmHandler);

    if (connectionParameters.role == LlTx) {
        unsigned char SET[5] = {FLAG, A_SENDER, C_SET, A_SENDER ^ C_SET, FLAG};
        unsigned char byteread;
        State state = START;
        int attempts = 0;

        while (attempts < connectionParameters.nRetransmissions) {
            printf("Attempt %d\n", attempts);
            write(fd, SET, 5);
            alarm(connectionParameters.timeout);

            while (!alarmFlag && state != STOP) {
                int bytesRead = read(fd, &byteread, 1);
                if (bytesRead > 0) {
                    printf("Byte read: %x\n", byteread);
                    switch (state) {
                        case START:
                            if (byteread == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byteread == A_RECEIVER) state = A_RCV;
                            else state = START;
                            break;
                        case A_RCV:
                            if (byteread == C_UA) state = C_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byteread == (A_RECEIVER ^ C_UA)) state = BCC_OK;
                            else state = START;
                            break;
                        case BCC_OK:
                            if (byteread == FLAG) state = STOP;
                            else state = START;
                            break;
                    }
                }
            }

            alarm(0);
            if (state == STOP) {
                printf("Connection established\n");
                nrtries = connectionParameters.nRetransmissions;
                timeout = connectionParameters.timeout;
                return fd;
            } else {
                attempts++;
                alarmFlag = 0;
                state = START;
            }
        }

        printf("Connection failed\n");
        return -1;
    } else if (connectionParameters.role == LlRx) {
        unsigned char UA[5] = {FLAG, A_SENDER, C_UA, A_SENDER ^ C_UA, FLAG};
        unsigned char byteread;
        State state = START;

        while (state != STOP) {
            int bytesRead = read(fd, &byteread, 1);
            if (bytesRead > 0) {
                printf("Byte read: %x\n", byteread);
                switch (state) {
                    case START:
                        if (byteread == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byteread == A_SENDER) state = A_RCV;
                        else state = START;
                        break;
                    case A_RCV:
                        if (byteread == C_SET) state = C_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byteread == (A_SENDER ^ C_SET)) state = BCC_OK;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byteread == FLAG) state = STOP;
                        else state = START;
                        break;
                }
            }
        }

        printf("Connection established\n");
        write(fd, UA, 5);
        return fd;
    }

    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(int fd, const unsigned char *buf, int bufSize, int sequenceN)
{
    if(fd = -1){return -1;}
    unsigned char frame[MAX_PAYLOAD + 6], stuffedBuf[MAX_PAYLOAD];
    int stuffedSize = 0;
    frame[0] = FLAG;
    frame[1] = A_SENDER;
    frame[2] = sequenceN == 0 ? C_I0 : C_I1;
    frame[3] = frame[1] ^ frame[2];
    unsigned char bcc2 = 0;
    byteStuffing(buf, &bufSize, &stuffedBuf, &stuffedSize);
    for(int i = 0; i < stuffedSize; i++){
        bcc2 ^= stuffedBuf[i];
        frame[4+i] = stuffedBuf[i];
    }
    frame[4+stuffedSize] = bcc2;
    frame[5+stuffedSize] = FLAG;

    int attempts = 0;
    State state = START;

    while (attempts < nrtries){
        printf("Sending frame...\n");
        write(fd, frame, stuffedSize+6);
        alarm(timeout);
        unsigned char adress, control, byte;
        while(!alarmFlag && state != STOP){
            int bytesRead = read(fd, &byte, 1);
            if(bytesRead > 0){
                switch(state){
                    case START:
                        if(byte == FLAG) {state = FLAG_RCV};
                        break;
                    case FLAG_RCV:
                        if(byte == A_RECEIVER) {state = A_RCV; adress = byte;}
                        else state = START; 
                        break;
                    case A_RCV:
                        if(byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1) {state = C_RCV; control = byte;}
                        else state = START;
                        break;
                    case C_RCV:
                        if(byte == (adress^control)){state = BCC_OK;}
                        else state = START;
                        break;
                    case BCC_OK:
                        if(byte == FLAG){state = STOP;}
                        else state = START;
                        break;
                }
            }
        }
        alarm(0);

        if(control == (sequenceN == 0 ? C_RR0 : C_RR1)){
            printf("Frame acknowledged\n");
            sequenceN++;
            return (stuffedSize+6);
        }else if(control == (sequenceN == 0 ? C_REJ0 : C_REJ1)){
            printf("Frame rejeted\n");
        }
        printf("Retrying frame... (%d/%d)\n", attempts + 1, nrtries);
        attempts++;
        alarmFlag = 0;
    }
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(int fd, unsigned char *packet, int *sequenceN)
{
    unsigned char byte, adress, control, bcc2, lastbyte, destuffed[MAX_PAYLOAD];
    State state = START;
    size_t i = 0;
    int destuffedSize = 0;
    while(state != STOP){
        int bytesRead = read(fd, &byte, 1);
        if(bytesRead > 0){
            switch(state){
                case START:
                    if(byte == FLAG){state = FLAG_RCV;}
                    break;
                case FLAG_RCV:
                    if(byte == A_SENDER){state = A_RCV; adress = byte;}
                    else state = START;
                    break;
                case A_RCV:
                    if(byte == C_I0 || byte == C_I1){state = C_RCV; control = byte;}
                    else state = START;
                    break;
                case C_RCV:
                    if(byte == (adress^control)){state = BCC_OK;}
                    else state = START;
                    break;
                case BCC_OK:
                    if(byte == FLAG){state = STOP; i--;}
                    else {packet[i] = byte; i++; lastbyte = byte;}
                    break;
            }
        }
    }
    byteDeStuffing(packet, &i, &destuffed, &destuffedSize);
    for(size_t j = 0; j < i ; j++){
        bcc2 ^= packet[j];
    }
    memcpy(packet, destuffed, destuffedSize);

    if (state == STOP && bcc2 == lastbyte){
        unsigned char rrframe[5] = {FLAG, A_RECEIVER, *sequenceN == 0 ? C_RR0 : C_RR1, (A_RECEIVER^(*sequenceN == 0 ? C_RR0 : C_RR1)), FLAG};
        write(fd, rrframe, 5);
        return (destuffedSize);
    }else{
        unsigned char rejframe[5] = {FLAG, A_RECEIVER, *sequenceN == 0 ? C_REJ0 : C_REJ1, (A_RECEIVER^(*sequenceN == 0 ? C_REJ0 : C_REJ1)), FLAG};
        write(fd , rejframe, 5);
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int fd, LinkLayer connectionParameters, int showStatistics)
{
    if(fd == -1){return -1;}
    if(connectionParameters.role == LlTx){
        unsigned char DISC[5] = {FLAG, A_SENDER, C_DISC, A_SENDER^C_DISC, FLAG};
        unsigned char byteread;
        State state = START;
        int attempts = 0;
        while (attempts < nrtries){
            printf("Sending DISC...\n");
            write(fd, DISC, 5);
            alarm(timeout);
            while(!alarmFlag && state != STOP){
                int bytesRead = read(fd, &byteread, 1);
                if(bytesRead > 0){
                    switch(state){
                        case START:
                            if(byteread == FLAG) {state = FLAG_RCV;}
                            break;
                        case FLAG_RCV:
                            if(byteread == A_RECEIVER) {state = A_RCV;}
                            else state = START;
                            break;
                        case A_RCV:
                            if(byteread == C_DISC) {state = C_RCV;}
                            else state = START;
                            break;
                        case C_RCV:
                            if(byteread == (A_RECEIVER^C_DISC)) {state = BCC_OK;}
                            else state = START;
                            break;
                        case BCC_OK:
                            if(byteread == FLAG) {state = STOP;}
                            else state = START;
                            break;
                    }
                }
            }
            alarm(0);
            if(state == STOP){
                printf("DISC acknowledged\n");
                unsigned char UA[5] = {FLAG, A_SENDER, C_UA, A_SENDER^C_UA, FLAG};
                write(fd, UA, 5);
                close(fd);
                return 1;
            }
            printf("Retrying DISC... (%d/%d)\n", attempts + 1, nrtries);
            attempts++;
            alarmFlag = 0;
            state = START;
            
        }
    }
    
    else if(connectionParameters.role == LlRx){
        unsigned char DISC[5] = {FLAG, A_RECEIVER, C_DISC, A_RECEIVER^C_DISC, FLAG};
        unsigned char byteread;
        State state = START;
        while(state != STOP){
            int bytesRead = read(fd, &byteread, 1);
            if(bytesRead > 0){
                switch(state){
                    case START:
                        if(byteread == FLAG) {state = FLAG_RCV;}
                        break;
                    case FLAG_RCV:
                        if(byteread == A_SENDER) {state = A_RCV;}
                        else state = START;
                        break;
                    case A_RCV:
                        if(byteread == C_DISC) {state = C_RCV;}
                        else state = START;
                        break;
                    case C_RCV:
                        if(byteread == (A_SENDER^C_DISC)) {state = BCC_OK;}
                        else state = START;
                        break;
                    case BCC_OK:
                        if(byteread == FLAG) {state = STOP;}
                        else state = START;
                        break;
                }
            }
        }
        if(state == STOP){
            printf("DISC received\n");
            unsigned char UA[5] = {FLAG, A_RECEIVER, C_DISC, A_RECEIVER^C_DISC, FLAG};
            write(fd, UA, 5);
            close(fd);
            return 1;
        }
    }
    

    return -1;
}
