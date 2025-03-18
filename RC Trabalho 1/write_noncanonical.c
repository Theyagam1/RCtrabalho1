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

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1
#define FLAG 0x7E
#define A_SENDER 0x03
#define A_RECIEVER 0x01
#define C_SENDER 0x03
#define C_RECIEVER 0x07

#define BUF_SIZE 256

#define FALSE 0
#define TRUE 1
#define ALARMCOUNT_MAX 4
#define ALARMTIME 3

int alarmEnabled = FALSE;
int alarmCount = 0;


typedef enum {
    START,
    STOP,
    A_RCV,
    C_RCV,
    F_RCV,
    BCC_OK
} State;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
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
    (void)signal(SIGALRM, alarmHandler);

    // Create string to send
    unsigned char buf[BUF_SIZE] = {0};

    buf[0] = FLAG;
    buf[1] = A_SENDER;
    buf[2] = C_SENDER;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    // In non-canonical mode, '\n' does not end the writing.
    // Test this condition by placing a '\n' in the middle of the buffer.
    // The whole buffer must be sent even with the '\n'.
    while (alarmCount < ALARMCOUNT_MAX){
        int bytes = write(fd, buf, BUF_SIZE);
        printf("%d bytes written\n", bytes);
        for (size_t i = 0; i < BUF_SIZE; i++)
        {
            printf("buf[%ld] = 0x%2X\n", i, buf[i]);
        }
        if (alarmEnabled == FALSE){
            alarm(ALARMTIME);
            alarmEnabled = TRUE;    
        }
        unsigned char byte;    
        State state = START;
        unsigned char adress, control, bcc;
        while (state != STOP){
            // Returns after 5 chars have been input
            int bytes = read(fd, &byte, 1);
          //  buf[bytes] = '\0'; // Set end of string to '\0', so we can printf
            printf("Estou no while\n");
            printf("byte: %u\n", byte);
            if(bytes > 0){
                switch(state){
                    case START:
                        if(byte == FLAG) state = F_RCV;
                        break;
                    case F_RCV:
                        if(byte == A_RECIEVER) {state = A_RCV; adress = byte;}
                        else state = START;
                        break;
                    case A_RCV:
                        if(byte == C_RECIEVER) {state = C_RCV; control = byte;}
                        else state = START;
                        break;
                    case C_RCV:
                        if(byte == (adress^control)) {state = BCC_OK; bcc = byte;}
                         else state = START;
                        break;                
                    case BCC_OK:
                        if(byte == FLAG) state = STOP;
                        else state = START;
                        break;
               }
            }
            break;
        }
        
        
        if(state == STOP){
            printf("Recieved successful\n");
            alarmCount = 4;
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
