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

typedef enum {
    START,
    STOP,
    A_RCV,
    C_RCV,
    F_RCV,
    BCC1_OK,
    BCC2_OK
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

int openConnection(int fd) {
    unsigned char uaFrame[5] = {FLAG, A_RECEIVER, C_UA, (A_RECEIVER^C_UA), FLAG};
    unsigned char byte, adress, control, bcc1;
    State state = START;

    while(state != STOP){
        
        int bytesRead = read(fd, &byte, 1);
        if(bytesRead > 0){
            printf("byte: %u\n", byte);
            switch(state){
                case START:
                	if(byte == FLAG) {state = F_RCV;}
					break;
				case F_RCV:
					if(byte == A_SENDER) {state = A_RCV; adress = byte;}
					else state = START;
					break;
				case A_RCV:
					if(byte == C_SET) {state = C_RCV; control = byte;}
					else state = START;
					break;
				case C_RCV:
					if(byte == (adress ^ control)){state = BCC1_OK; bcc1 = byte;}
					else state = START;		
					break;
				case BCC1_OK:
					if(byte == FLAG) {state = STOP;}
					else state = START;
					break;
			}
		}
	}
	if(state == STOP){
		int bytes = sendFrame(fd, uaFrame, 5);
		return 1;
	}

	return -1;
}

int frameRead(int fd, unsigned char *data, int *sequenceNumber) {
    unsigned char byte, adress, control, bcc1, bcc2, lastbyte;
    State state = START;
    size_t i = 0;
    printf("Reading frame...\n");
    while (state != STOP) {
        int bytesRead = read(fd, &byte, 1);
        if (bytesRead > 0) {
            printf("byte: %u\n", byte);
            switch (state) {
                case START:
                    if (byte == FLAG) {state = F_RCV;}
                    break;
                case F_RCV:
                    if (byte == A_SENDER) {state = A_RCV; adress = byte;} 
                    else {state = START;}
                    break;
                case A_RCV:
                    if (byte == C_I0 || byte == C_I1) {state = C_RCV; control = byte;}
                    else {state = START;}
                    break;
                case C_RCV:
                    if (byte == (adress ^ control)) {state = BCC1_OK; bcc1 = byte;} 
                    else {state = START;}
                    break;
                case BCC1_OK:
                    if (byte == FLAG) {state = STOP; lastbyte = data[i]; i--;} 
                    else {data[i] = byte; i++;}
                    break;
            }
        }
    }
    for (size_t j = 0; j < i; j++)
    {
        bcc2 ^= data[j];
    }
    printf("BCC2: %u\n", bcc2);
    
    if (state == STOP && bcc2 == lastbyte) {
        unsigned char rrFrame[5] = {FLAG, A_RECEIVER, *sequenceNumber == 0 ? C_RR0 : C_RR1, (A_RECEIVER ^ (*sequenceNumber == 0 ? C_RR0 : C_RR1)), FLAG};
        sendFrame(fd, rrFrame, 5);
        return i;
    }
    else {
        unsigned char rejFrame[5] = {FLAG, A_RECEIVER, *sequenceNumber == 0 ? C_REJ0 : C_REJ1, (A_RECEIVER ^ (*sequenceNumber == 0 ? C_REJ0 : C_REJ1)), FLAG};
        sendFrame(fd, rejFrame, 5);
    }
    return -1;
}
/*int connectionClose(int fd) {
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
*/
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
    newtio.c_cc[VMIN] = 5;  // Blocking read until 5 chars received
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

    while (openConnection(fd) < 0) {
		
    }

    unsigned char data[BUF_SIZE];
    int sequenceNumber = 0;
    int retry = 0;

    while (sequenceNumber < 2 || retry < MAX_RETRIES) {
        int bytesRead = frameRead(fd, data, &sequenceNumber);
        if (bytesRead > 0) {
            printf("Received frame\n");
        } else {
            printf("Failed to read frame\n");
        }
    }

    /*if (connectionClose(fd) < 0) {
        printf("Failed to close connection\n");
    }*/

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
