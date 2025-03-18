// Read from serial port in non-canonical mode
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

int alarmTriggered = 0;

void alarmHandler(int signal) {
    alarmTriggered = 1;
}

void sendFrame(int fd, unsigned char *frame, int size) {
    write(fd, frame, size);
    tcflush(fd, TCOFLUSH);
}

int openConnection(int fd) {
    unsigned char setFrame[5];
    unsigned char uaFrame[5] = {FLAG, A_RECEIVER, C_UA, A_RECEIVER ^ C_UA, FLAG};

    for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
        int bytesRead = read(fd, setFrame, 5);
        if (bytesRead == 5 && setFrame[2] == C_SET && setFrame[3] == (setFrame[1] ^ setFrame[2])) {
            sendFrame(fd, uaFrame, 5);
            printf("Connection established!\n");
            return 1;
        }

        printf("Retrying llopen... (%d/%d)\n", attempt + 1, MAX_RETRIES);
    }

    printf("Failed to establish connection\n");
    return -1;
}

int frameRead(int fd, unsigned char *data, int *sequenceNumber) {
    unsigned char frame[BUF_SIZE + 6];
    unsigned char response[5];
    int bytesRead = read(fd, frame, BUF_SIZE + 6);

    if (bytesRead < 6) {
        printf("Incomplete frame received\n");
        return -1;
    }

    if (frame[0] != FLAG || frame[bytesRead - 1] != FLAG) {
        printf("Invalid frame format\n");
        return -1;
    }

    unsigned char BCC1 = frame[1] ^ frame[2];
    if (frame[3] != BCC1) {
        printf("BCC1 error\n");
        return -1;
    }

    unsigned char BCC2 = 0;
    for (int i = 4; i < bytesRead - 2; i++) {
        BCC2 ^= frame[i];
    }

    if (frame[bytesRead - 2] != BCC2) {
        printf("BCC2 error\n");
        response[2] = (*sequenceNumber == 0) ? C_REJ0 : C_REJ1;
        sendFrame(fd, response, 5);
        return -1;
    }

    memcpy(data, &frame[4], bytesRead - 6);
    *sequenceNumber = (*sequenceNumber + 1) % 2;

    response[0] = FLAG;
    response[1] = A_RECEIVER;
    response[2] = (*sequenceNumber == 0) ? C_RR0 : C_RR1;
    response[3] = response[1] ^ response[2];
    response[4] = FLAG;

    sendFrame(fd, response, 5);
    return bytesRead - 6;
}

int connectionClose(int fd) {
    unsigned char discFrame[5];
    unsigned char uaFrame[5] = {FLAG, A_RECEIVER, C_UA, A_RECEIVER ^ C_UA, FLAG};

    int bytesRead = read(fd, discFrame, 5);
    if (bytesRead == 5 && discFrame[2] == C_DISC && discFrame[3] == (discFrame[1] ^ discFrame[2])) {
        sendFrame(fd, uaFrame, 5);
        close(fd);
        return 1;
    }
    return -1;
}

int main(int argc, char *argv[]) {
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];
    if (argc < 2) {
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
    if (fd < 0) {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;
    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
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
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }
    printf("New termios structure set\n");

    if (openConnection(fd) < 0) {
        close(fd);
        return 1;
    }

    unsigned char data[BUF_SIZE];
    int sequenceNumber = 0;

    while (TRUE) {
        int bytesRead = frameRead(fd, data, &sequenceNumber);
        if (bytesRead > 0) {
            printf("Received frame with %d bytes\n", bytesRead);
        } else {
            printf("Failed to read frame\n");
            break;
        }
    }

    if (connectionClose(fd) < 0) {
        printf("Failed to close connection\n");
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
