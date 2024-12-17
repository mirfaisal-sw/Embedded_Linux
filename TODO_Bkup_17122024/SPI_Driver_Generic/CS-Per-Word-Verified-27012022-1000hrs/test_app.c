#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>

#define DEVNAME         "/dev/spi0"

static int fd;
uint8_t tx_buf[1024];

void sendCommandToTuner(uint8_t *packetBuf);
void printHexBytesForDebugging(uint8_t *buf);

int open_file()
{
	fd = open (DEVNAME, O_RDWR);
	if (fd < 0 )
	{
		printf ("Cannot open SPI device file\n");
                return -1;
	}

}

void sendBytesToTuner(uint8_t *packetBuf, uint8_t cnt)
{
	uint32_t ret_cnt;

	ret_cnt = write(fd,packetBuf,cnt);
	if(cnt < 0){
		printf("Write operation fialed\n");
	}else{
		printf("No of bytes written = %d\n",cnt);
	}
}

int main(int argc, char *argv[])
{
	uint32_t milliSecDelay = 0;

	/*Handle command line argument*/
	if(argc <= 1){
		printf("Usage : <program_name> <delay_in_ms>\n");
		return -1;
	}

	milliSecDelay = atoi(argv[1]);
        printf("ms delay be  - %d\n",milliSecDelay);
 
	open_file();
	memset(tx_buf,0x55,512);
	
	while(1)
	{	
		sendBytesToTuner(tx_buf,16);
		printf("Command transmitted\n");
	        usleep(milliSecDelay*1000);
	}

	close(fd);

	return 0;
}

