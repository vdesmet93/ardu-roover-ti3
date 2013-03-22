#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "commands.h"
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>

#define ARG_POS_INT 0x3B
#define ARG_NEG_INT 0x1B
#define ARG_STRING  0x2B

int number,n, fd;

void read() 
{
	char *buf = (char *) malloc(sizeof(char) * (255*255));
	n = read( fd, buf, 255 * 255 );

	if ( n == -1 )
		printf ( "Error = %s\n", strerror( errno ) );

	printf ( "Number of bytes read = %i\n", n );
	for(int i = 0; i < n; i++) 
	{
		printf(" %x ", *(buf++));
	}
}

/*
* Checksum generator for the buffer which will be send
*/
int getChecksum(unsigned char* buf) 
{
	int i; 
	unsigned char n; 
	int c = 0; 
	i = 3; 
	n = buf[2] - 2; 
	while (n > 1) 
	{ 
		c += ((unsigned char)buf[i]<<8) | (unsigned char)buf[i+1]; 
		c = c & 0xffff; 
		n -= 2; 
		i += 2; 
	} 
	if (n > 0) 
		c = c ^ (int)((unsigned char) buf[i]); 
	return c;
}

/**
* Send a packet without arguments.
* For example, SYNC0, OPEN, RESET, CLOSE
*/
void sendPacket(char command) 
{
	// create array. Length = 6 because we don't have any arguments
	unsigned char buf[6];
	// header
	buf[0] = 0xFA; // 250
	buf[1] = 0xFB; // 251
	// length
	buf[2] = 3; // command + 2 byte checksum
	// command
	buf[3] = command;
	// generate checksum
	int checksum = getChecksum(buf);
	// parse the checksum
	char high = (checksum & 0xFF00) >> 8;
	char low = checksum & 0x00FF;
	// add checksum to array, high byte is first
	buf[4] = high;
	buf[5] = low;
	// send the array, print resultcode
	int resultcode = write (fd, buf, 6);
	printf("Resultcode: %d\n", resultcode);
}

/*
* Handler which is called by the system when the program needs to be closed
* Emergency stop the robot, reset the robot and close the connection
*/
void my_handler(int s)
{
	if(s == 2) 
	{
		sendPacket(E_STOP);
		sleep(1);
		sendPacket(RESET);
		usleep(10000);
		sendPacket(CLOSE);
		usleep(10000);
	}
	exit(1); 
}

/*
* Send a packet with 1 argument: ENABLE, SETA, SONAR
*/
void sendPacket(char command, int argument) 
{
	// create array. Length = 9: 2 header, 1 byte count, 1 command, 1 arg. type, 2 argument, 
	//														2 checksum
	unsigned char buf[9];
	/* ================= HEADER ================= */
	buf[0] = 0xFA; // 250
	buf[1] = 0xFB; // 251

	/* =============== PACKET LENGTH =============== */
	// command + 2 byte checksum + 2 byte message
	buf[2] = 6; 

	/* ================= COMMAND ================= */
	buf[3] = command;

	/* ============== ARGUMENT TYPE ============== */
	if(argument > 0)
		buf[4] = (unsigned char) ARG_POS_INT;
	else
		buf[4] = (unsigned char) ARG_NEG_INT;

	/* ================= ARGUMENT ================= */
	// parse argument high and low
	char highArgument = (argument & 0xFF00) >> 8;
	char lowArgument = (argument & 0x00FF);

	// add argument to array. 
	// Low byte first(instead of high-first for checksum, because f*ck consistency)
	buf[5] = lowArgument;
	buf[6] = highArgument;

	/* ================= CHECKSUM ================= */
	// generate checksum
	int checksum = getChecksum(buf);

	// parse the checksum
	char highChecksum = (checksum & 0xFF00) >> 8;
	char lowChecksum = checksum & 0x00FF;

	// add checksum to array, high byte is first
	buf[7] = highChecksum;
	buf[8] = lowChecksum;

	/* ================= TRANSFER ================= */	
	// send the array, print resultcode
	int resultcode = write (fd, buf, 9);
	printf("Resultcode: %d\n", resultcode);
}
/*
* Send a packet with a string as argument
*/
void sendPacket(char command, char* argument, int size) {
	// create array. Length = 9: 2 header, 1 byte count, 1 command, 1 arg. type, 2 argument, 
	//														2 checksum
	int bytes = size + 7;

	unsigned char buf[bytes];

	/* ================= HEADER ================= */
	buf[0] = 0xFA; // 250
	buf[1] = 0xFB; // 251

	/* =============== PACKET LENGTH =============== */
	// command + 2 byte checksum + 2 byte message
	buf[2] = 6; 

	/* ================= COMMAND ================= */
	buf[3] = command;

	/* ============== ARGUMENT TYPE ============== */
	buf[4] = (unsigned char) ARG_STRING;

	/* ================= ARGUMENT ================= */
	buf[5] = size;

	// parse argument high and low
	printf("parsing\n");
	char c = *argument;
	int idx = 6;
	while(c != '\0' && (idx - 6) != size) 
	{
		printf("P: %c\n", c);
		buf[idx] = c;
		argument++;
		idx++;
		c = *argument;
	}

	/* ================= CHECKSUM ================= */
	// generate checksum
	int checksum = getChecksum(buf);

	// parse the checksum
	char highChecksum = (checksum & 0xFF00) >> 8;
	char lowChecksum = checksum & 0x00FF;

	// add checksum to array, high byte is first
	buf[bytes-2] = highChecksum;
	buf[bytes-1] = lowChecksum;

	/* ================= TRANSFER ================= */	
	// send the array, print resultcode
	int resultcode = write (fd, buf, bytes);
	printf("Resultcode: %d\n", resultcode);
}

/*
* The Pioneer requires a pulse message every two seconds
* We're sending it each second
*/
void pulse(void*) 
{
	while(1) 
	{
		sendPacket(PULSE);
		sleep(1);
	}
}
void startThread(void) 
{
	pthread_t t;
	void *pulse(void *);	

	pthread_create(&t, NULL, pulse, NULL);
}

int open_port(void)
{

	fd = open("/dev/ttyUSB0", O_RDWR | O_NDELAY | O_SYNC);
	if (fd == -1)
		perror("open_port: Unable to open /dev/ttyUSB0 - ");
	else
	{
		printf("Port Opened successfully\n");

		struct termios options;
		tcgetattr(fd, &options);

		// set settings
		cfsetispeed(&options, B9600);
		cfsetospeed(&options, B9600);
		options.c_cflag |= (CLOCAL | CREAD);
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		options.c_iflag &= ~(IXON | IXOFF | IXANY);
		options.c_oflag &= ~OPOST;

		if ( tcsetattr( fd, TCSANOW, &options ) == -1 )
			printf ("Error with tcsetattr = %s\n", strerror ( errno ) );
		else
			printf ( "%s\n", "tcsetattr succeed" );

		fcntl(fd, F_SETFL, FNDELAY);

		// send SYNC0, SYNC1, SYNC2, OPEN to open the connection
		sleep(1);
		sendPacket(SYNC0);
		sleep(1);
		read();
		sendPacket(SYNC1);
		sleep(1);
		read();
		sendPacket(SYNC2);
		sleep(1);
		read();
		startThread();

		sendPacket(OPEN);
		sleep(1);
		// at this point, the connection is opened
	}

	return (fd);
}


int main(void)
{
	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	open_port();

	// disable sonar
	sendPacket(SONAR, 0);
	usleep(10000);

	// enable the motors	
	sendPacket(ENABLE, 1);
	sleep(1);

	// rotate
	sendPacket(DHEAD, 180);
	sleep(3);

	// start driving
	sendPacket(VEL, 1000);
	sleep(1);
	while(1) 
	{
		read();
		sleep(1);
	}
	return 0; 
}	
