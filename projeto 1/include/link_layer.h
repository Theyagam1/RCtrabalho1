// Link layer header.
// NOTE: This file must not be changed.

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_

typedef enum
{
    LlTx,
    LlRx,
} LinkLayerRole;

typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

// SIZE of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 1000

// MISC
#define FALSE 0
#define TRUE 1

/**
 * Abre uma conexão usando os parâmetros definidos na estrutura LinkLayer.
 * 
 * @param connectionParameters Estrutura contendo os parâmetros da conexão.
 * @return Retorna "1" em caso de sucesso ou "-1" em caso de erro.
 */
int llopen(LinkLayer connectionParameters);

/**
 * Envia dados no buffer `buf` com tamanho `bufSize`.
 * 
 * @param fd Descritor de arquivo da conexão serial.
 * @param buf Buffer contendo os dados a serem enviados.
 * @param bufSize Tamanho do buffer (número de bytes a serem enviados).
 * @param sequenceN Número de sequência do quadro (0 ou 1).
 * @return Retorna o número de caracteres escritos ou "-1" em caso de erro.
 */
int llwrite(int fd, const unsigned char *buf, int bufSize, int sequenceN);
/**
 * Recebe dados no buffer `packet`.
 * 
 * @param fd Descritor de arquivo da conexão serial.
 * @param packet Buffer onde os dados recebidos serão armazenados.
 * @param sequenceN Ponteiro para o número de sequência esperado (0 ou 1).
 * @return Retorna o número de caracteres lidos ou "-1" em caso de erro.
 */
int llread(int fd, unsigned char *packet, int *sequenceN);

/**
 * Fecha a conexão previamente aberta.
 * 
 * @param showStatistics Se TRUE, imprime estatísticas no console ao fechar.
 * @return Retorna "1" em caso de sucesso ou "-1" em caso de erro.
 */
int llclose(int fd, LinkLayer connectionParameters, int showStatistics);

/**
 * Perform byte stuffing on the input data.
 * @param input Input data.
 * @param inputsize Size of the input data.
 * @param output Output buffer for stuffed data.
 * @param outputsize Size of the output data.
 */
void byteStuffing(unsigned char *input, int *inputsize, unsigned char *output, int *outputsize);

/**
 * Perform byte destuffing on the input data.
 * @param input Input data with stuffing.
 * @param inputsize Size of the input data.
 * @param output Output buffer for destuffed data.
 * @param outputsize Size of the output data.
 */
void byteDeStuffing(unsigned char *input, int *inputsize, unsigned char *output, int *outputsize);


#endif // _LINK_LAYER_H_
