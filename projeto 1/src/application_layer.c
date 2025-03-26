// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParameters;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;
    if (strcmp(role, "tx") == 0)
    {
        connectionParameters.role = LlTx;
    }
    else if (strcmp(role, "rx") == 0)
    {
        connectionParameters.role = LlRx;
    }
    else
    {
        printf("Invalid role\n");
        return;
    }
    strcpy(connectionParameters.serialPort, serialPort);

    int fd = llopen(connectionParameters);
    if (fd == -1)
    {
        printf("Failed to establish connection\n");
        return;
    }

    if(connectionParameters.role == LlTx){
        FILE *file = fopen(filename, "r");
        if (file == NULL)
        {
            printf("Failed to open file\n");
            return;
        }
        printf("File opened...\n");

        unsigned char buffer[MAX_PAYLOAD_SIZE];
        int sequenceNw = 0;
        size_t bufSize = 0;
        fgets((char *)buffer, MAX_PAYLOAD_SIZE, file);
        bufSize = strlen((char*)buffer);
        fclose(file);
        int bytesSent = llwrite(fd, buffer, bufSize, sequenceNw);
        if (bytesSent == -1){
            printf("Failed to send frame\n");
            return;
        }
        printf("Frame sent\n");
        buffer[0] = 0;
        sequenceNw = 1;
        bytesSent = llwrite(fd, buffer, 1, sequenceNw);
        if (bytesSent == -1){
            printf("Failed to send frame\n");
            return;
        }
        printf("Frame sent\n");
    }
    else if (connectionParameters.role == LlRx){
        FILE *file = fopen (filename, "wb");
        if (file == NULL)
        {
            printf("Failed to open file\n");
            return;
        }
        printf("File opened...\n");
        unsigned char bufferR[MAX_PAYLOAD_SIZE];
        int sequenceN = 0;
        while (sequenceN < 2){
            printf("Waiting for frame...\n");
            int bytesRead = llread(fd, bufferR, sequenceN);
            if (bytesRead != -1){
                printf("Frame received\n");
                if(sequenceN == 1){
                    fwrite(bufferR, 1, bytesRead, file);
                    fclose(file);
                }
            }
            sequenceN++;
        } 

    }

    if (llclose(fd, connectionParameters) == -1)
    {
        printf("Failed to close connection\n");
        return;
    }

}
