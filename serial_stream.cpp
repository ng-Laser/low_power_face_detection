#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <string>    // for to_string on file names
#include <time.h>    // for timing transmission of pics

uint8_t inbuf[2];
uint8_t outbuf;

int main(int argc, char** argv){
    /* cli args parse */
    int hi_res = 0;
    int help = 0;
    char *tty_path = NULL;
    int index;
    int c;
    opterr = 0;
    while ((c = getopt (argc, argv, "lht:")) != -1) {
        switch (c) {
            case 'l':
                hi_res = 1;
                break;
            case 'h':
                help = 1;
                break;
            case 't':
                tty_path = optarg;
                break;
            case '?':
                if (optopt == 't')
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint(optopt))
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
                exit(1);
            default:
                abort();
        }
    }

    if (help) {
        printf("usage: %s [-lh] [-t tty]\n", argv[0]);
        printf("-l\t: use high resolution (326x244)\n");
        printf("-h\t: print help message\n");
        printf("-t tty\t: set path to tty device file\n");
        exit(0);
    }

    int USB;
    if (tty_path) {
        USB = open( tty_path, O_RDWR|O_NOCTTY|O_NONBLOCK );
    }
    else {
        USB = open( "/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NONBLOCK );
    }
    printf("USB = %d\n", USB);

    if( USB==-1 ){
        printf("serial port open bad\n");
        exit(1);
    }
    else{
        printf("serial port open good\n");
    }

    if (fcntl(USB, F_SETFL, 0) == -1)
    {
        printf("can't set tty device non-blocking\n");
        exit(1);
    }

    struct termios tty;
    memset (&tty, 0, sizeof tty);

    if( tcgetattr(USB, &tty)!= 0){
        printf("Error fetching tty config, %i\n",errno);
    }
    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B230400); //460800);  //  115200); 
    cfsetispeed (&tty, (speed_t)B230400); //460800);  //  115200); 

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] =  100;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;


    /* Flush Port, then applies attributes */
    tcflush( USB, TCIFLUSH );
    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
        printf("Error configuring tty, %i\n", errno);
    }

    FILE* outfile;
    int marker_state = 0;
    int intransit = 0;
    int pic_state = 0;

    int pic_counter = 0;
    /* keep reading until start of frame */
    while (1)
    {
        read(USB, inbuf, 1);
        if (inbuf[0] == 'A') {
            marker_state += 1;
        }
        else {
            marker_state = 0;
        }
        if (marker_state == 16) {
            printf("Synced\n");
            marker_state = 0;
            intransit = 1;
            outfile = fopen("output.bmp", "wb");
            break;
        }
    }
    clock_t start_time;
    while (1)
    {
        read(USB, inbuf, 2);
        if (!intransit) {
            if (inbuf[0] == 'A' && inbuf[1] == 'A') {
                marker_state += 2;
            }
            if (marker_state == 16) {
                printf("Start of frame\n");
                outfile = fopen("output.bmp", "wb");
                intransit = 1;
                start_time = clock();
                marker_state = 0;
            }
        }
        else {
            outbuf = ((inbuf[0] & 0xF) << 4)  | (inbuf[1] & 0xF);
            // outbuf = ((inbuf[0] & 0xF) << 4); //  | (inbuf[1] & 0xF);
            // outbuf = ((raw & 0x1) << 7) | ((raw & 0x2) << 5) | ((raw & 0x4) << 3) | ((raw & 0x8) << 1);
            fwrite(&outbuf, sizeof(uint8_t), 1, outfile);
            pic_state += 1;
            if(pic_state == 20008 && (!hi_res)) {
              clock_t end_time = clock();
              printf("%f", (double)(end_time - start_time)/CLOCKS_PER_SEC);
              pic_state = 0;
              intransit = 0;
              fclose(outfile);
              system(("echo 'y' | ffmpeg -vcodec rawvideo -f rawvideo  -s 164x122 \
                      -i output.bmp -f image2 -vcodec png output" + std::to_string(pic_counter++) + ".png &> /dev/null").c_str());
              printf("Dumped\n");
            }
            else if (pic_state == 79544 && hi_res) {
              clock_t end_time = clock();
              printf("%f", (double)(end_time - start_time)/CLOCKS_PER_SEC);
              pic_state = 0;
              intransit = 0;
              fclose(outfile);
              system(("echo 'y' | ffmpeg -vcodec rawvideo -f rawvideo -pix_fmt gray -s 326x244 \
                     -i output.bmp -f image2 -vcodec png output" + std::to_string(pic_counter++) + ".png &> /dev/null").c_str());
              printf("Dumped\n");
            }
        }
    }

    return 0;
}
