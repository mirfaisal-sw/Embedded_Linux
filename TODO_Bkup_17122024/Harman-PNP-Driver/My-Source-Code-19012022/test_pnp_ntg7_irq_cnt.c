#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>

#define REG_IRQ_NUM             _IOW('a','a',int32_t*)
#define REG_IRQ_DELAY           _IOW('b','b',int32_t*)
#define TUNER_CHIP_IRQ_NUM       119

#define DEVNAME         "/dev/spi0"
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define RESET_GPIO     		52
#define BOOTMODE1_GPIO 		56
#define BOOTMODE2_GPIO 		82

#define CHECKSUM_SIZE           4
#define DAB_MAX_PKT_SIZE        2032   //Data length
#define DAB_MSG_HEADER_LEN      12   //Header length
#define BUF_SIZE                (DAB_MAX_PKT_SIZE + DAB_MSG_HEADER_LEN + CHECKSUM_SIZE)
#define FIFO_SIZE               (4 * BUF_SIZE)

#define THREAD
#define GET_CONFIG_CMD_ID       0x03

static int fd, fd_irq_counter;
static uint32_t irq_cnt;
uint8_t usr_buf[4];
static int irq_number;
static uint8_t tx[] = { 0x57, 0x44, 0x52, 0x00, 0x04, 0x00,
			0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
			0xF2, 0x00, 0x00, 0x00};

static uint8_t get_config_tx_buf[] = { 0x57, 0x44, 0x52, 0x00, 0x03, 0x00, 
				       0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 
				       0xF1, 0x00, 0x00, 0x00};    
					
uint8_t rx[1024];
uint32_t milliSecDelay;
uint32_t gSuccessfulReadCount = 0;

uint8_t *readResponse();
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

	#if 1
	fd_irq_counter = open("/dev/irq_counter_device", O_RDWR);
        if(fd_irq_counter < 0) {
            printf("Cannot open IRQ_CNT device file...\n");
            return -1;
        }
	#endif

	return 0;
}

int register_irqnum_to_monitor(int fd, int32_t irq_num)
{
	if (ioctl(fd_irq_counter, REG_IRQ_NUM,(int32_t*) &irq_num))
	{
                printf("IRQ ioctl Failed\n");
                close(fd);
                return -1;
        }

	return 0;
}

int setDelayInThreadedBH(int fd, int32_t delay_ms)
{
	if (ioctl(fd, REG_IRQ_DELAY,(int32_t*) &delay_ms))
        {
                printf("PNP Delay ioctl Failed\n");
                close(fd);
                return -1;
        }

        return 0;

}

int set_logic_of_pin(int a, int level)
{
	char dir[50], logic[50]; /*exp[50]*/
	/*sprintf(exp, "echo %d > /sys/class/gpio/export", a);
	system(exp);
	printf("Exported GPIO %d\n", a);
	*/
	sprintf(dir, "echo out > /sys/class/gpio/gpio%d/direction", a);
	system(dir);
	printf("Set direction of GPIO %d as out\n", a);
	
	sprintf(logic, "echo %d > /sys/class/gpio/gpio%d/value", level, a);
	system(logic);
	printf("Set GPIO %d to logic %d \n", a, level);
	
	return 0;
}

int generate_reset()
{
	set_logic_of_pin(RESET_GPIO, 0);
	usleep(100000);
	set_logic_of_pin(RESET_GPIO, 1);
	printf("Resetting done");
	return 0;
}


int set_bootmode(char a)
{
	switch(a) 
	{
		case '1':
			set_logic_of_pin(BOOTMODE1_GPIO, 0);
			set_logic_of_pin(BOOTMODE2_GPIO, 1);
			printf("Setting Normal bootmode");
			break;
			
		case '2':
			set_logic_of_pin(BOOTMODE1_GPIO, 1);
			set_logic_of_pin(BOOTMODE2_GPIO, 0);
			printf("Setting SWDL bootmode");
			break;
			
		default:
			printf("Enter Valid bootmode");
                        break;
        }		
	return 0;
}
	
int transfer_cmd()
{		
	
	int i;
	char option;
		
	printf("Enter the Option\n");
	printf("1. Read \n");
	printf("2. Write \n");
	printf("3. Exit \n");
	scanf(" %c", &option);
	//option = '1';
	printf("Option = %c\n", option);
		
	switch(option) {
			
		case '1':
			read(fd, rx, ARRAY_SIZE(rx));
			for( i = 0; i < ARRAY_SIZE(rx); i++ )
       			{
               			printf("Data= %c\n", rx[i]);
       			}
                              
               		printf("Data Read Done!\n\n");
			break;

		case '2':
			write(fd, tx, ARRAY_SIZE(tx));
               		printf("Data Write Done!\n\n");

			read(fd, rx, ARRAY_SIZE(rx));
                        for( i = 0; i < ARRAY_SIZE(rx); i++ )
                        {
                        	printf("Data= %x\n", rx[i]);
                        }

		        printf("Data Read Done!\n\n");

			break;

		case '3':
			close(fd);				
			break;

		default:
			printf("Enter Valid option");
                break;
	}
	return 0;
}
	
void *write_function(void *arg)
{
	while(1)
	{	
		printf("In function - %s\n",__func__);
		//write(fd, tx, ARRAY_SIZE(tx));

		sendCommandToTuner(get_config_tx_buf);
		usleep(milliSecDelay*1000);
	}
	printf("Data Write Done!\n\n");
	return NULL;
}

void *read_function(void *arg)
{
	int i;

	uint8_t *rx_buf; 
	
	while(1)
	{
		/*do{
		}
		while(flag == 0);*/
		
		gSuccessfulReadCount++;

		printf("Read Count - %d\n",gSuccessfulReadCount);
		printf("Data received be - \n");

		rx_buf = readResponse();
       		printHexBytesForDebugging(rx_buf);
		
		free(rx_buf);
		printf("\n");
		//pthread_mutex_unlock(&lock);
		//flag = 0;
	}
	printf("Data Read Done!\n\n");
	
	return NULL;
}

void write_m_times_read_n_times(int m_cnt, int n_cnt)
{
	int i = 0,j = 0;

	for(i = 0;i<m_cnt;i++)
	{
		printf("Write loop count - %d\n",i+1);
		write(fd,tx,ARRAY_SIZE(tx));	
		usleep(milliSecDelay*1000);
	}

	sleep(1);	

	#if 1
	for(j = 0;j<n_cnt;j++)
	{
		printf("Read loop count - %d\n",j+1);
		read(fd, rx, 20);
                printf("Data received be - \n");
                for( i = 0; i < 20; i++ )
                {
                        printf("%x  ", rx[i]);
                }

                printf("\n");

	}
	#endif

}

void *read_irq_stat(void *argp)
{
	while(1)
        {

                read(fd_irq_counter,usr_buf,4);
                //printf("%x %x %x %x\n",usr_buf[0],usr_buf[1],usr_buf[2],usr_buf[3]);
                memcpy(&irq_cnt,usr_buf,4);
		printf("---------------------------------------\n");
                printf("IRQs received on IRQ Num - %d be - %d\n",TUNER_CHIP_IRQ_NUM,irq_cnt);
		printf("---------------------------------------\n");
                sleep(1);
        }

        return NULL;
}

void printHexBytesForDebugging(uint8_t *buf)
{
	uint8_t i;
	uint16_t len = ((buf[9] << 8)|buf[8]);
        printf("Received %d bytes are - \n",len);
	for(i = 0;i<len;i++)
	{	
		if(i%8 == 0)
			printf("\n");

		printf("%x ",buf[i]);
	}

	printf("\n");
}

//get_config_tx_buf
void sendCommandToTuner(uint8_t *packetBuf)
{
//	write(fd,packetBuf,ARRAY_SIZE(packetBuf));
	write(fd,packetBuf,16);
}

uint8_t *readResponse()
{
	uint8_t header_buf[12], *buf;
	uint16_t len;

	/*First read 12 bytes header*/
	read(fd,header_buf,12);

	/*Decode length field from header*/
	len = ((header_buf[9] << 8) | header_buf[8]);

	/*Read Data payload and 4 bytes checksum.*/
        buf = malloc(12 + len + 4);

	read(fd,buf+12,len+4);

        memcpy(buf,header_buf,12);
	
	return buf;
}	

int main(int argc, char *argv[])
{
	char boot;
	pthread_t tid1, tid2, threadID3;
	struct sched_param params;
	params.sched_priority = 90;//sched_get_priority_max(SCHED_FIFO);

	int ret;
		
	/*Handle command line argument*/
	if(argc <= 1){
		printf("Usage : <program_name> <delay_in_ms>\n");
		return -1;
	}

	milliSecDelay = atoi(argv[1]);

        printf("ms delay be  - %d\n",milliSecDelay);
 
	set_logic_of_pin(RESET_GPIO, 1);
	
	printf("Choose bootmode:\n");
	printf("1. Normal mode\n");
	printf("2. SWDL mode\n");
        //boot = '1';
	scanf("%c", &boot);

	printf("Bootmode = %c\n", boot);
	set_bootmode(boot);
	
	generate_reset();
	open_file();
	//transfer_cmd();
	
   	sleep(1);/*Initial power on delay for Tuner chip to initialize.*/

	
	sendCommandToTuner(get_config_tx_buf);
	printf("Command transmitted\n");
	//sleep(100);
	while(1);

	#if 0
	/*For testing*/
	sendCommandToTuner(get_config_tx_buf);
	uint8_t *rx_buf = readResponse();
	printHexBytesForDebugging(rx_buf);
	free(rx_buf);
	while(1);
	#endif

	#if 1
	if(-1 == register_irqnum_to_monitor(fd_irq_counter, TUNER_CHIP_IRQ_NUM)){
		printf("IRQ registration for IRQ monitoring failed.\n");
		return -1;
	}
	#endif

	/*read initial 48 bytes*/
	read(fd, rx, 48);
        printHexBytesForDebugging(rx);
	read(fd, rx, 48);
        printHexBytesForDebugging(rx);
        
	printf("Data read done.\n");
	sleep(100);
	
	while(1);
	//setDelayInThreadedBH(fd,10000);

	#if 0
	int write_cnt, read_cnt;

 	write_cnt = (FIFO_SIZE/20);
	read_cnt = write_cnt;

	write_cnt = 1;
	read_cnt = 1;
	write_m_times_read_n_times(write_cnt, read_cnt);

	printf("#### M - N cnt read write done.####\n");
	while(1);
        #endif

	#ifdef THREAD
	pthread_create( &tid1, NULL, write_function, NULL );
        pthread_create( &tid2, NULL, read_function, NULL );
        pthread_create( &threadID3, NULL, read_irq_stat, NULL );
	
        ret = pthread_setschedparam(tid1, SCHED_FIFO, &params);
        if (ret != 0) {
         // Print the error
         printf("Unsuccessful in setting thread realtime prio, tid1\n");
         return -1;     
        }

	ret = pthread_setschedparam(threadID3, SCHED_FIFO, &params);
        if (ret != 0) {
         // Print the error
         printf("Unsuccessful in setting thread realtime prio, tid1\n");
         return -1;
        }

	ret = pthread_setschedparam(tid2, SCHED_FIFO, &params);
        if (ret != 0) {
         // Print the error
         printf("Unsuccessful in setting thread realtime prio, tid1\n");
         return -1;
        }

	printf("Threads created\n");
	pthread_join( tid1, NULL); 
	pthread_join( tid2, NULL); 
	pthread_join( threadID3, NULL);

 	#endif
	
	close(fd);

	return 0;
}

