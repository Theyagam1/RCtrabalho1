// Write to serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7E
#define A_SENDER 0x03
#define A_RECEIVER 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define C_I0 0x00
#define C_I1 0x40
#define C_RR0 0x05
#define C_RR1 0x85
#define C_REJ0 0x01
#define C_REJ1 0x81
#define MAX_RETRIES 3
#define TIMEOUT 3  // Timeout in seconds
#define BUF_SIZE 256

#define FALSE 0
#define TRUE 1

typedef enum {
    START, STOP, BCC1_OK, A_RCV, C_RCV, F_RCV, BCC2_OK
} State;

int alarmTriggered = 0;

void alarmHandler(int signal) {
    alarmTriggered = 1;
}

int sendFrame(int fd, unsigned char *frame, int size) {
    int bytes = write(fd, frame, size);
    printf("N bytes: %d\n", bytes);
    for(size_t j = 0; j<size; j++){
        printf("byte: %u\n", frame[j]);
    }
    return bytes;
}

void randomData(unsigned char *data, int length) {
    srand(time(NULL)); // Inicializa o gerador de números aleatórios

    for (int i = 0; i < length; i++) {
        data[i] = rand() % 256; // Gera um valor aleatório entre 0 e 255
    }
}

int openConection (int fd){
    unsigned char byte , adress, control, bcc1;
    unsigned char setFrame[5] = {FLAG, A_SENDER, C_SET, (A_SENDER ^ C_SET), FLAG};
    State state = START;

    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        int bytes = sendFrame(fd, setFrame, 5);
        alarm(TIMEOUT);
        
        while (!alarmTriggered) {
            int bytesRead = read(fd, &byte, 1);
            if(bytesRead > 0){
                printf("byte: %u\n", byte);
                switch (state){
                    case START:
                        if(byte == FLAG) state = F_RCV;
                        break;
                    case F_RCV:
                        if(byte == A_RECEIVER) {state = A_RCV; adress = byte;}
                        else state = START;
                        break;
                    case A_RCV:
                        if (byte == C_UA) {state = C_RCV; control = byte;}
                        else state = START;
                        break;
                    case C_RCV:
                        if(byte == (adress^control)) {state = BCC1_OK; bcc1 = byte;}
                        else state = START;
                        break;
                    case BCC1_OK:
                        if(byte == FLAG) {state = STOP;}
                        else state = START;
                        break;    
                }
            }
            if (state == STOP) break;
        }
        alarm(0);

        if (state == STOP) {
            printf("Connection established!\n");
            return 1;
        }

        printf("Retrying Connection... (%d/%d)\n", attempt + 1, MAX_RETRIES);
        alarmTriggered = 0;
    }
    printf("Failed to establish connection\n");
    return -1;
}


int frameWrite(int fd, unsigned char *data, int length, int sequenceNumber) {
    unsigned char frame[BUF_SIZE + 6];
    frame[0] = FLAG;
    frame[1] = A_SENDER;
    frame[2] = sequenceNumber == 0 ? C_I0 : C_I1;
    frame[3] = frame[1] ^ frame[2];  // BCC1

    unsigned char bcc2 = 0;
    for (int i = 0; i < length; i++) {
        bcc2 ^= data[i];
    }
    memcpy(&frame[4], data, length);

    frame[4 + length] = bcc2;
    frame[5 + length] = FLAG;

    int attempts = 0;
    size_t i = 0;

    State state = START;
    unsigned char adress, control, bcc1, lastbyte;
    while (attempts < MAX_RETRIES) {
        printf("Sending frame...\n");
        sendFrame(fd, frame, length + 6);
        alarm(TIMEOUT);
        unsigned char responsebyte;
        printf("Waiting for response...\n");
        while (!alarmTriggered){
            int bytesRead = read(fd, &responsebyte, 1);
            if (bytesRead > 0){
                printf("byte: %u\n", responsebyte);
                switch (state){
                    case START:
                        if(responsebyte == FLAG) state = F_RCV;
                        break;
                    case F_RCV:
                        if(responsebyte == A_SENDER) {state = A_RCV; adress = responsebyte;}
                        else state = START;
                        break;
                    case A_RCV:
                        if(responsebyte == C_RR0 || responsebyte == C_RR1 || responsebyte == C_REJ0 || responsebyte == C_REJ1) {state = C_RCV; control = responsebyte;}
                        else state = START;
                        break;
                    case C_RCV:
                        if(responsebyte == (adress^control)) {state = BCC1_OK; bcc1 = responsebyte;}
                        else state = START;
                        break;
                    case BCC1_OK:
                        if(responsebyte == FLAG) {state = STOP;}
                        else state = START;
                        break;
                }
            }
            if (state == STOP) break;
        }
        alarm(0);

        if (control == (sequenceNumber == 0 ? C_RR0 : C_RR1)) {
            printf("Frame acknowledged\n");
            return 1;
        } else if (control == (sequenceNumber == 0 ? C_REJ0 : C_REJ1)) {
            printf("Frame rejected\n");
        }            
        printf("Retrying frame... (%d/%d)\n", attempts + 1, MAX_RETRIES);
        attempts++;
        alarmTriggered = 0;
    }
    return -1;
}


int conectionClose(int fd) {
    unsigned char discFrame[5] = {FLAG, A_SENDER, C_DISC, A_SENDER ^ C_DISC, FLAG};
    sendFrame(fd, discFrame, 5);

    unsigned char response[5];
    int bytesRead = read(fd, response, 5);
    if (bytesRead == 5 && response[2] == C_DISC) {
        unsigned char uaFrame[5] = {FLAG, A_SENDER, C_UA, A_SENDER ^ C_UA, FLAG};
        sendFrame(fd, uaFrame, 5);
        close(fd);
        return 1;
    }
    return -1;
}

int main(int argc, char *argv[])
{

    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];
    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }
    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(serialPortName);
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

    signal(SIGALRM, alarmHandler);

    if(openConection(fd) < 0){
        close(fd);
        return 1;
    }

    unsigned char data[BUF_SIZE];
    int sequenceNumber = 0;

    while (sequenceNumber < 2) {
        randomData(data, BUF_SIZE);
        int result = frameWrite(fd, data, BUF_SIZE, sequenceNumber);

        if (result == 1) {
            printf("Frame acknowledged, sending next frame\n");
            sequenceNumber++;
        } else if (result < 0) {
            printf("Failed to write frame\n");
            break;
        }
    }



    // Wait until all bytes have been written to the serial port
    sleep(1);

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
