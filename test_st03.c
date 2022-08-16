#include <stdio.h> 
#include <unistd.h> 
#include <fcntl.h> 
#include <termios.h> 
#include <errno.h>
#include <string.h>

int fd;
unsigned short crcvalue;
unsigned char txBuffer[100];
unsigned char rxBuffer[100];
struct termios tty_attributes;

void crc16(unsigned short *crc, char c) {
	int i,j;

	for (i=0; i!=8; c>>=1, i++) {
		j = (c^(*crc)) & 1;
		(*crc)>>=1;

		if (j) {
			(*crc)^=0xa001;
		}
	}
}

int main(void) {
	int i;
	int rtc;

	fd=open("/dev/ttyUSB0",O_RDWR|O_NOCTTY|O_NONBLOCK);

	if (fd==-1) {
		printf ("Error no is : %d\n", errno);
		printf("Error description is : %s\n",strerror(errno));
		return(-1);
	} else {
        tcgetattr(fd,&tty_attributes);

        // c_cflag
        // Enable receiver
        tty_attributes.c_cflag |= CREAD;

        // 8 data bit
        tty_attributes.c_cflag |= CS8;

        // c_iflag
        // Ignore framing errors and parity errors.
        tty_attributes.c_iflag |= IGNPAR;

		tty_attributes.c_iflag&=~(INLCR|ICRNL);

		tty_attributes.c_iflag&=~(IXON|IXOFF);
        // c_lflag
        // DISABLE canonical mode.
        // Disables the special characters EOF, EOL, EOL2,
        // ERASE, KILL, LNEXT, REPRINT, STATUS, and WERASE, and buffers by lines.

        // DISABLE this: Echo input characters.
        tty_attributes.c_lflag &= ~(ICANON);

        tty_attributes.c_lflag &= ~(ECHO);

        // DISABLE this: If ICANON is also set, the ERASE character erases the preceding input
        // character, and WERASE erases the preceding word.
        tty_attributes.c_lflag &= ~(ECHOE);

        // DISABLE this: When any of the characters INTR, QUIT, SUSP, or DSUSP are received, generate the corresponding signal. 
        tty_attributes.c_lflag &= ~(ISIG);



        // Minimum number of characters for non-canonical read.
        tty_attributes.c_cc[VMIN]=1;

        // Timeout in deciseconds for non-canonical read.
        tty_attributes.c_cc[VTIME]=0;

        // Set the baud rate
        cfsetospeed(&tty_attributes,B19200);
        cfsetispeed(&tty_attributes,B19200);

        tcsetattr(fd, TCSANOW, &tty_attributes);
	}

	for (int counter=1;;counter++) {
		for (int address=18;address<=26;address++) {


			txBuffer[0]=0x54;			// SYNC
			txBuffer[1]=0b01010001;		// HDB2
			txBuffer[2]=0b01000001;		// HDB1
			txBuffer[3]=address;		// NODE ADDR
			txBuffer[4]=0x00;			// PC ADDR
			txBuffer[5]=0x02;			// COMANDO


			crcvalue=0;

			for (i=1;i<=5;i++) {
				crc16(&crcvalue,txBuffer[i]);
			}
			txBuffer[6]=(crcvalue>>8)&0x00FF;
			txBuffer[7]=(crcvalue)&0x00FF;
			
			//txBuffer[7]=0x12;

			for (i=0;i<=7;i++) {
				printf("%02X ",txBuffer[i]);
			}
			printf("  ");


			write(fd,txBuffer,8);
			tcdrain(fd);
			tcflush(fd,TCIFLUSH);

			usleep(1000*1000);

			for (i=0;i<=20;i++) {
				rxBuffer[i]=0;
			}

			rtc=read(fd,rxBuffer,13);

			for (i=0;i<=13;i++) {
				printf("%02X ",rxBuffer[i]);
			}

			// Estrae temperatura e umidita'

			unsigned int iTem = ((rxBuffer[6]*256)&0xFF00)+((rxBuffer[7])&0xFF);
			unsigned int iUmi = ((rxBuffer[8]*256)&0xFF00)+((rxBuffer[9])&0xFF);

			float temp = -40 + 0.01 * iTem;
			float umid = -4 + 0.0405 * iUmi + -0.0000028 * (float)iUmi * (float)iUmi;

			printf(" A=%03d (0x%02x) ",address,address);

			printf(" T=%.1f H=%.0f ",temp,umid);

			printf("\n");

		}
	}
	close(fd);
}


// https://www.acmesystems.it/serial
// https://www.i-programmer.info/programming/cc/10027-serial-c-and-the-raspberry-pi.html?start=2
