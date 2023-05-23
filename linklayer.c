#include "linklayer.h"

//Realizado por Gonçalo Coelho (up202005368) e Marco Moreira (up202004135)

//Constantes definidas para os valores hexadecimais usados no protocolo de comunicação
#define F 0x5c
#define A0 0x01
#define A1 0x03
#define C_SET 0x03
#define C_DISC 0x0B
#define C_UA 0x07
#define C_RR0 0x01
#define C_RR1 0x21
#define C_REJ0 0x05
#define C_REJ1 0x25
#define C_I0 0x00
#define C_I1 0x02
#define ESCAPE 0X5d
#define STUFF_BYTE 0x20

int STOP, TIMEOUT, fd;
struct termios oldtio; 

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    END
} State;

void stop_alarm() {  //Função acionada quando o tempo definido no alarme chega ao fim
   
    TIMEOUT = TRUE;
}

void start_alarm(int sec) {  //Função que implementa o alarme/temporizador
    
    TIMEOUT = FALSE;
    signal(SIGALRM, stop_alarm);
    alarm(sec);
}

int send_frame(unsigned char* frame, int size, int fd) {  //Função que envia uma trama pela porta serie
   
    int send = write(fd, frame, size);
    if(send == -1) {
        perror("write");
        return -1;
    }

    return send;
}

int receive_frame(unsigned char* frame, int fd) {  //Função que recebe uma trama pela porta serie
    
    int receive, i = 0, count = 0;
    State state = START;
    unsigned char byte;

    //Implementa uma máquina de estados que termina quando chega no estado END
    while(state != END) {  
        receive = read(fd, &byte, 1);  //Lê um byte
        if(receive == -1) {
            return -1;
        }

        switch(state) {
            case START:
                //Caso o byte lido for uma FLAG, guarda e avança para o estado FLAG_RCV
                if(byte == F) {  
                    frame[i++] = byte;
                    state = FLAG_RCV;
                }
                break;

            case FLAG_RCV:
                //Caso o byte lido coincidir com um dos endereços, guarda e avança para o estado A_RCV
                if(byte == A0 || byte == A1) {  
                    frame[i++] = byte;
                    state = A_RCV;
                } 
                //Caso o byte lido não for uma Flag ou um endereço, retorna ao estado START
                else if(byte != F) {  
                    i = 0;
                    state = START;
                }
                break;

            case A_RCV:
                //Caso o byte lido coincidir com um campo de controlo válido, guarda e avança para o estado C_RCV
                if(byte == C_SET || byte == C_DISC || byte == C_UA || byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1 || byte == C_I0 || byte == C_I1) {
                    frame[i++] = byte;
                    state = C_RCV;
                } 
                //Caso o byte lido for uma Flag, retorna ao estado FLAG_RCV
                else if(byte == F) {
                    i = 1;
                    state = FLAG_RCV;
                } 
                //Caso o byte lido não for uma Flag ou um campo de controlo, retorna ao estado START
                else {
                    i = 0;
                    state = START;
                }
                break;

            case C_RCV:
                //Caso o byte lido coincidir com o XOR entre o endereço e o campo de controlo, guarda e avança para o estado BCC_OK
                if(byte == (frame[1] ^ frame[2])) {
                    frame[i++] = byte;
                    state = BCC_OK;
                } 
                //Caso o byte lido for uma Flag, retorna ao estado FLAG_RCV
                else if(byte == F) {
                    i = 1;
                    state = FLAG_RCV;
                } 
                //Caso o byte lido não for uma Flag ou BCC válido, retorna ao estado START
                else {
                    state = START;
                }
                break;
            
            case BCC_OK:
                //Caso o byte lido for uma Flag, avança para o estado END
                if(byte == F) {
                    frame[i++] = byte;
                    state = END;
                } 
                //Enquanto o byte lido não for uma Flag e não tiver excedido o tamanho máximo permitido, guarda
                else if(i < 2 * MAX_PAYLOAD_SIZE + 5) {
                    frame[i++] = byte;
                }
                //Caso o byte lido não for uma Flag e já tiver excedido o tamanho máximo permitido, retorna ao estado START
                else {
                    state = START;
                }
                break;

            case END:
                break;

            default:
                break;
        }
    }

    return i;
}

int llopen(linkLayer connectionParameters) {
    
    int receive, send, count = 0;
    unsigned char set[5], ua[5];
    struct termios newtio;
    STOP = FALSE;

    //Abre a ligação através da porta serie
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if(fd < 0) {
        perror("open");
        return -1;
    }

    //Guarda as especificações antigas da ligação
    if(tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        return -1;
    }

    //Configura as especificações novas da ligação
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = 0;

    //Limpa a ligação
    if(tcflush(fd, TCIOFLUSH) == -1) {
        perror("tcflush");
        return -1;
    }

    //Altera as especificaões da ligação para as novas
    if(tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    //MODO EMISSOR
    if(connectionParameters.role == 0) {
        set[0] = F;
        set[1] = A0;
        set[2] = C_SET;
        set[3] = A0 ^ C_SET;
        set[4] = F;

        //Envia a trama SET
        send = send_frame(set, 5, fd);
        if(send < 0) {
            return -1;
        }
        printf("SET sent\n");

        //Inicia o temporizador
        start_alarm(connectionParameters.timeOut);
        while(STOP == FALSE) {
            //Recebe a trama
            receive = receive_frame(ua, fd);

            //Caso o alarme chegue ao fim, é verificado se se deve retransmitir a trama ou acabar a ligação
            if(TIMEOUT == TRUE) {
                if(count < connectionParameters.numTries) {
                    send = send_frame(set, 5, fd);
                    if(send < 0) {
                        return -1;
                    }

                    start_alarm(connectionParameters.timeOut);
                    count++;
                }
                else {
                    printf("time out\n");
                    return -1;
                }
            }

            //Caso a trama recebida corresponda à trama UA, termina o ciclo de leitura
            if(receive > 0 && ua[0] == F && ua[1] == A0 && ua[2] == C_UA && ua[3] == (A0 ^ C_UA) && ua[4] == F) {
                printf("UA received\n");

                STOP = TRUE;
                start_alarm(0);
            }
        }   
    } 
    //MODO RECETOR
    else if(connectionParameters.role == 1) {
        ua[0] = F;
        ua[1] = A0;
        ua[2] = C_UA;
        ua[3] = A0 ^ C_UA;
        ua[4] = F;

        while(STOP == FALSE) {
            //Recebe a trama
            receive = receive_frame(set, fd);

            //Caso a trama recebida corresponda à trama SET, envia a trama UA e termina o ciclo de leitura
            if(receive > 0 && set[0] == F && set[1] == A0 && set[2] == C_SET && set[3] == (A0 ^ C_SET) && set[4] == F) {
                printf("SET received\n");

                send = send_frame(ua, 5, fd);
                if(send < 0) {
                    return -1;
                }
                printf("UA sent\n");

                STOP = TRUE;
            }
        }
    }

    return 0;
}

int llwrite(char* buf, int bufSize) {

    unsigned char frame[2 * MAX_PAYLOAD_SIZE + 6], r[5], bcc2;
    int frameSize = 0, receive, send, i;
    STOP = FALSE;

    //Guarda o cabeçalho da trama I
    frame[frameSize++] = F;
    frame[frameSize++] = A0;
    frame[frameSize++] = C_I0;
    frame[frameSize++] = A0 ^ C_I0;

    //Determina o valor de BCC2
    bcc2 = buf[0];
    for (i = 1; i < bufSize; i++) {
        bcc2 ^= buf[i];
    }

    //Caso os bytes sejam uma Flag ou byte de Escape, realiza Stuffling e guarda na trama. Se não for o caso, guarda na trama 
    for (i = 0; i < bufSize; i++) {
        if(buf[i] == F || buf[i] == ESCAPE) {
            frame[frameSize++] = ESCAPE;
            frame[frameSize++] = buf[i] ^ STUFF_BYTE;
        }
        else {
            frame[frameSize++] = buf[i];
        }
    }

    //Caso o BCC2 seja uma Flag ou byte de Escape, realiza Stuffling e guarda na trama. Se não for o caso, guarda na trama
    if(bcc2 == F || bcc2 == ESCAPE) {
        frame[frameSize++] = ESCAPE;
        frame[frameSize++] = bcc2 ^ STUFF_BYTE;
    } 
    else {
        frame[frameSize++] = bcc2;
    }

    //Guarda a Flag para indicar o fim da trama
    frame[frameSize++] = F;

    //Envia a trama
    send = send_frame(frame, frameSize, fd);
    if(send < 0) {
        return -1;
    }
    printf("first frame I sent\n");

    while(STOP == FALSE) {
        //Recebe a trama
        receive = receive_frame(r, fd);

        //Caso a trama recebida corresponda à trama RR0, envia a segunda trama I.        
        if(receive > 0 && r[0] == F && r[1] == A0 && r[2] == C_RR0 && r[3] == (A0 ^ C_RR0) && r[4] == F) {
            printf("RR0 received\n");

            //Altera o valor do controlo e BCC1 para ser identificada como a segunda trama I
            frame[2] = C_I1;
            frame[3] = A0 ^ C_I1;

            send = send_frame(frame, frameSize, fd);
            if(send < 0) {
                return -1;
            }
            printf("second frame I sent\n");
        } 
        //Caso a trama recebida corresponda à trama REJ0, reenvia a trama I
        else if(receive > 0 && r[0] == F && r[1] == A0 && r[2] == C_REJ0 && r[3] == (A0 ^ C_REJ0) && r[4] == F) {
            printf("REJ0 received\n");

            send = send_frame(frame, frameSize, fd);
            if(send < 0) {
                return -1;
            }
            printf("first frame I resent\n");
        } 
        //Caso a trama recebida corresponda à trama RR1, termina o ciclo de leitura
        else if(receive > 0 && r[0] == F && r[1] == A0 && r[2] == C_RR1 && r[3] == (A0 ^ C_RR1) && r[4] == F) {
            printf("RR1 received\n");
            
            STOP = TRUE;
        } 
        //Caso a trama recebida corresponda à trama REJ1, reenvia a segunda trama I
        else if(receive > 0 && r[0] == F && r[1] == A0 && r[2] == C_REJ1 && r[3] == (A0 ^ C_REJ1) && r[4] == F) {
            printf("REJ1 received\n");

            send_frame(frame, frameSize, fd);
            printf("second frame I resent\n");
        }
    }

    return frameSize;
}

int llread(char* packet) {

    unsigned char frame[2 * MAX_PAYLOAD_SIZE + 6], rr[5], rej[5], bcc2;
    int receive, i, send, frameSize = 0;
    STOP = FALSE;

    while(STOP == FALSE) {
        //Recebe a trama
        receive = receive_frame(frame, fd);

        //Caso a trama recebida corresponda à trama I, verifica o BCC2 e envia uma trama RR0 ou REJ0
        if(receive > 0 && frame[0] == F && frame[1] == A0 && frame[2] == C_I0 && frame[3] == (A0 ^ C_I0) && frame[receive - 1] == F) {
            printf("first frame I received\n");

            //Caso os bytes sejam um byte de Escape, realiza de-stuffling e guarda na packet. Se não for o caso, guarda na packet 
            for(i = 4; i < receive - 2; i++) {
                if(frame[i] == ESCAPE) {
                    if(i == receive - 3) {
                        frame[receive - 2] ^= STUFF_BYTE;
                    } 
                    else {
                        packet[frameSize++] = frame[++i] ^ STUFF_BYTE;
                    }
                } 
                else {
                    packet[frameSize++] = frame[i];
                }
            }

            //Determina o valor de BCC2
            bcc2 = packet[0];
            for (i = 1; i < frameSize; i++) {
                bcc2 ^= packet[i];
            }

            //Verifica se o valor de BCC2, corresponde ao enviado. Caso se verifique, envia uma trama RR0. Caso contráro, envia uma trama REJ0
            if(bcc2 != frame[receive - 2]) {
                rej[0] = F;
                rej[1] = A0;
                rej[2] = C_REJ0;
                rej[3] = A0 ^ C_REJ0;
                rej[4] = F;

                send = send_frame(rej, 5, fd);
                if(send < 0) {
                    return -1;
                }
                printf("REJ0 sent\n");
            } 
            else {
                rr[0] = F;
                rr[1] = A0;
                rr[2] = C_RR0;
                rr[3] = A0 ^ C_RR0;
                rr[4] = F;

                send = send_frame(rr, 5, fd);
                if(send < 0) {
                    return -1;
                }
                printf("RR0 sent\n");
            }
        }
        //Caso a trama recebida corresponda à segunda trama I, verifica o BCC2 e enviar uma trama RR1 ou REJ1
        if(receive > 0 && frame[0] == F && frame[1] == A0 && frame[2] == C_I1 && frame[3] == (A0 ^ C_I1) && frame[receive - 1] == F) {
            printf("second frame I received\n");

            //Caso necessário, realiza de-stuffling do BCC2
            if(frame[receive - 3] == ESCAPE) {
                frame[receive - 2] ^= STUFF_BYTE;
            }

            //Verifica se o valor de BCC2, corresponde ao enviado. Caso se verifique, envia uma trama RR1. Caso contráro, envia uma trama REJ1
            if(bcc2 != frame[receive - 2]) {
                rej[0] = F;
                rej[1] = A0;
                rej[2] = C_REJ1;
                rej[3] = A0 ^ C_REJ1;
                rej[4] = F;

                send = send_frame(rej, 5, fd);
                if(send < 0) {
                    return -1;
                }
                printf("REJ1 sent\n");
            } 
            else {
                rr[0] = F;
                rr[1] = A0;
                rr[2] = C_RR1;
                rr[3] = A0 ^ C_RR1;
                rr[4] = F;

                send = send_frame(rr, 5, fd);
                if(send < 0) {
                    return -1;
                }
                printf("RR1 sent\n");

                STOP = TRUE;
            }
        }
    }

    return frameSize;
}

int llclose(linkLayer connectionParameters, int showStatistics) {
    unsigned char disc[5], ua[5], cv[5];
    int receive, send;
    STOP = FALSE;

    //MODO EMISSOR
    if(connectionParameters.role == 0) {
        disc[0] = F;
        disc[1] = A0;
        disc[2] = C_DISC;
        disc[3] = A0 ^ C_DISC;
        disc[4] = F;

        ua[0] = F;
        ua[1] = A1;
        ua[2] = C_UA;
        ua[3] = A1 ^ C_UA;
        ua[4] = F;

        //Envia a trama DISC
        send = send_frame(disc, 5, fd);
        if(send < 0) {
            return -1;
        }
        printf("DISC sent\n");

        while(STOP == FALSE) {
            //Recebe a trama
            receive = receive_frame(disc, fd);

            //Caso a trama recebida corresponda à trama DISC, envia a trama UA e termina o ciclo de leitura
            if(receive > 0 && disc[0] == F && disc[1] == A1 && disc[2] == C_DISC && disc[3] == (A1 ^ C_DISC) && disc[4] == F) {
                printf("DISC received\n");

                send = send_frame(ua, 5, fd);
                if(send < 0) {
                    return -1;
                }
                printf("UA sent\n");

                STOP = TRUE;
            }
        }
    } 
    //MODO RECETOR
    else if(connectionParameters.role == 1) {
        disc[0] = F;
        disc[1] = A1;
        disc[2] = C_DISC;
        disc[3] = A1 ^ C_DISC;
        disc[4] = F;

        while(STOP == FALSE) {
            //Recebe a trama
            receive = receive_frame(cv, fd);

            //Caso a trama recebida corresponda à trama DISC, envia a trama DISC e volta ao ciclo de leitura
            if(receive > 0 && cv[0] == F && cv[1] == A0 && cv[2] == C_DISC && cv[3] == (A0 ^ C_DISC) && cv[4] == F) {
                printf("DISC received\n");

                send = send_frame(disc, 5, fd);
                if(send < 0) {
                    return -1;
                }
                printf("DISC sent\n");
            } 
            //Caso a trama recebida corresponda à trama UA, termina o ciclo de leitura
            else if(receive > 0 && cv[0] == F && cv[1] == A1 && cv[2] == C_UA && cv[3] == (A1 ^ C_UA) && cv[4] == F) {
                printf("UA received\n");
                STOP = TRUE;
            }
        }
    }

    //Limpa a ligação
    if(tcflush(fd, TCIOFLUSH) == -1) {
        perror("tcflush");
        return -1;
    }

    //Altera as especificações da ligação para as antigas
    if(tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    //Fecha a ligação
    if(close(fd) == -1) {
        perror("close");
        return -1;
    }

    return 0;
}
