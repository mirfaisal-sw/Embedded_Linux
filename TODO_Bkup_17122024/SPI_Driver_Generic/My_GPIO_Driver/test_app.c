/***************************************************************************//**
*  \file       test_app.c
*
*  \details    Userspace application to test the Device driver
*
* *******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <signal.h>
 
#define REG_CURRENT_TASK _IOW('a','a',int32_t*)
 
#define SIGMIR 29
 
int doneCtrlC = 0;
int irqSigDriver = 0;
 
void ctrl_c_handler(int n, siginfo_t *info, void *unused)
{
    if (n == SIGINT) {
        printf("\nrecieved ctrl-c\n");
        doneCtrlC = 1;
    }
}
 
void sig_event_handler(int n, siginfo_t *info, void *unused)
{
    if (n == SIGMIR) {
        irqSigDriver = info->si_int;
        printf ("Received signal from kernel : Value =  %u\n", irqSigDriver);
    }
}
 
int main()
{
    int fd;
    int32_t value, number;
    uint8_t dummyReading = 0;
    struct sigaction act;
 
    printf("*********************************\n");
    printf("*********************************\n");
 
    /* install ctrl-c interrupt handler to cleanup at exit */
    sigemptyset (&act.sa_mask);
    act.sa_flags = (SA_SIGINFO | SA_RESETHAND);
    act.sa_sigaction = ctrl_c_handler;
    sigaction (SIGINT, &act, NULL);
 
    /* install custom signal handler */
    sigemptyset(&act.sa_mask);
    act.sa_flags = (SA_SIGINFO | SA_RESTART);
    act.sa_sigaction = sig_event_handler;
    sigaction(SIGMIR, &act, NULL);
 
    printf("Installed signal handler for SIGMIR = %d\n", SIGMIR);
 
    printf("\nOpening Driver\n");
    fd = open("/dev/gpio_device", O_RDWR);
    if(fd < 0) {
            printf("Cannot open device file...\n");
            return 0;
    }
 
    printf("Registering application ...");
    /* register this task with kernel for signal */
    if (ioctl(fd, REG_CURRENT_TASK,(int32_t*) &number)) {
        printf("Failed\n");
        close(fd);
        exit(1);
    }
    printf("Done!!!\n");
   
    while(1)
    {
        printf("Sensor1 reading = %d\n",dummyReading);
	usleep(250000);

        printf("Sensor2 reading = %d\n",(dummyReading+1));
	usleep(250000);
	
        if(irqSigDriver == 1)
	{
		irqSigDriver = 0;
		printf("Accelerometer reading = %d\n",(dummyReading + 2));
	}

    	if(doneCtrlC == 1){
		break;
	}

	dummyReading++;
    }
 
    printf("Closing Driver\n");
    close(fd);
}
