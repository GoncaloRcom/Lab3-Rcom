#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define TRUE 1
#define FALSE 0
#define BAUDRATE B9600
#define F 0x7E
#define A 0x03
#define CVSET 0x03
#define CVUA 0x07

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    END,
    STOP
} FState;

int main(int argc, char** argv){

    int r_open;

    if(argc < 2) {
        printf("Usage: recetor <SERIALPORT>");
        exit(1);
    }

    if(r_open = llopen(argv[1]) == -1){
        perror("llopen");
        exit(-1);
    }
    
    if(r_open == 1) printf("Connected!\n");

    return 0;
}

int llopen(char* port) {

    int fd, byte, set = FALSE, i = 0;
    unsigned char buffer[256];
    struct termios oldtio, newtio;

    fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) { 
        perror("open"); 
        exit(-1); 
    }

    if (tcgetattr(fd,&oldtio) == -1) {
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = 0;

    if(tcflush(fd, TCIOFLUSH) == -1) {
        perror("tcflush");
        exit(-1);
    }

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    FState fstate = START;
    while(set == FALSE){
        if ((byte = read(fd, &buffer[i], 1)) == -1) {
            perror("read");
            exit(-1);
        }
        state_machine(&fstate, buffer[i]);
        if (fstate == END) {
            set = TRUE;
        }
        i++;
    }

    if(set == TRUE){
        buffer[0] = F;
        buffer[1] = A;
        buffer[2] = CVUA;
        buffer[3] = A ^ CVUA;
        buffer[4] = F;

        write(fd, buffer, 5);

        return 1;
    }
    
    return -1;
}

void state_machine(FState *fstate, unsigned char byte) {
    
    switch (*fstate) {
        case START:
            if (byte == F) {
                *fstate = FLAG_RCV;
            }
            break;
        case FLAG_RCV:
            if (byte == A) {
                *fstate = A_RCV;
            } 
            else if (byte != F) {
                *fstate = START;
            }
            break;
        case A_RCV:
            if (byte == CVSET) {
                *fstate = C_RCV;
            } else if (byte == F) {
                *fstate = FLAG_RCV;
            } else {
                *fstate = START;
            }
            break;
        case C_RCV:
            if (byte == (A ^ CVSET)) {
                *fstate = BCC_OK;
            } else if (byte == F) {
                *fstate = FLAG_RCV;
            } else {
                *fstate = START;
            }
            break;
        case BCC_OK:
            if (byte == F) {
                *fstate = STOP;
            } else {
                *fstate = START;
            }
            break;
        case STOP:
            break;
    }
}