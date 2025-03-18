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
#define A_SEENDER 0x03
#define A_RECEIVER 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define C_RR0 0x05
#define C_RR1 0x85
#define C_REJ0 0x01
#define C_REJ1 0x81

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int alarmFlag = 0;
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
        unsigned char SET[5] = {FLAG, A_SEENDER, C_SET, A_SEENDER ^ C_SET, FLAG};
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
        unsigned char UA[5] = {FLAG, A_SEENDER, C_UA, A_SEENDER ^ C_UA, FLAG};
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
                        if (byteread == A_SEENDER) state = A_RCV;
                        else state = START;
                        break;
                    case A_RCV:
                        if (byteread == C_SET) state = C_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byteread == (A_SEENDER ^ C_SET)) state = BCC_OK;
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
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    return 1;
}
