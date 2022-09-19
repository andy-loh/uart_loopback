/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Test UART loopback from userspace.
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>

 *
 * Date: 08/09/2022
 * Contact: Teh Yu Sheng(yusheng.teh@starfivetech.com)
 * 	    Tan Chun Hau (chunhau.tan@starfivetech.com)

**************************************************************************
 * @objective 	Check the spi driver's ability to read and write from registers
 * @source: 	https://github.com/kushalembsys/uart-loopback-test/blob/master/uart-loopback.c
 *
 * @params 	(char) the driver that to be tested (e.g /dev/ttyS3)
 *	   	^^Note** This need be configured in device tree (pin 0 for rx, pin 2 for tx)
 *         	(unsigned int) the baud rate that needs to be used
 * @returns 	0 for success, < 0 if failed;
*	   	details return number can be found at
*	   	testcases/ddt/utils/user/st_defines.h
*@example 	./uart_loopback /dev/ttyS3 9600
*		When pin 0 and 2 are available and connected to each other,
*          	return 0 upon success.
*          	When pin 0 does not receiving correct amount of signals,
*          	return -1.
*/

/************************************************************************/

#include <stdlib.h>
#include <termios.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>

// File control definitions
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
//#include <st_defines.h>
//#include <st_log.h>

#define COUNT 5
#define SUCCESS 0
#define FAILURE -1

extern int errno ;

pthread_barrier_t barrier_work_main;
const unsigned char Tx_Data[COUNT] = {'A','1','B','2','C','A','1','B','2','C'};

// declare the timeval struct to calculate the time required to read the bytes
struct timeval tval_before, tval_after[COUNT], tval_result[COUNT];

// A struct that passes parameters into a function that is used
// by thread.
struct Device {
	int devicename;
	unsigned int baudrate;
}uart_device;

void* write_bytes(void* device)
{
	struct Device *device_ptr = ( struct Device*) device;
	int fd = device_ptr->devicename;
	unsigned int baudrate = device_ptr->baudrate;

	// synchronize main thread and worker thread
	if (write(fd, Tx_Data, COUNT) != COUNT) {
		printf("error in write\n");
	}
	gettimeofday(&tval_before, NULL);
	pthread_barrier_wait(&barrier_work_main);
}

speed_t convertIntToSpeedType(unsigned int baud_rate)
{
	switch (baud_rate)
	{
		case 50		: return B50;
		case 75		: return B75;
		case 110	: return B110;
		case 134	: return B134;
		case 150	: return B150;
		case 200	: return B200;
		case 300	: return B300;
		case 600	: return B600;
		case 1200   	: return B1200;
		case 2400   	: return B2400;
		case 4800   	: return B4800;
		case 9600   	: return B9600;
		case 19200  	: return B19200;
		case 38400  	: return B38400;
		case 57600  	: return B57600;
		case 115200 	: return B115200;
		case 230400 	: return B230400;
		case 460800 	: return B460800;
		case 921600		: return B921600;
		case 1000000 	: return B1000000;
		case 1152000 	: return B1152000;
		case 2000000 	: return B2000000;
		case 2500000 	: return B2500000;
		case 3000000 	: return B3000000;
		case 4000000 	: return B4000000;
		default:
			return B0;
	}
}

int main(int argc, char const *argv[])
{
	int err = FAILURE;
	// get the device from user input
	char sDevice[128];
	strcpy(sDevice, argv[1]);
	uart_device.devicename = open(sDevice,  O_RDWR | O_NOCTTY | O_NONBLOCK);
	printf("Executing UART LOOPBACK test for %s\n\n", sDevice);
        if (uart_device.devicename < 0) {
                printf("io failed to close device");
		goto end;
	}

	struct termios l_sUARTConfig, l_sDefaultUARTConfig;
	if (tcgetattr(uart_device.devicename, &l_sUARTConfig) != 0
	 || tcgetattr(uart_device.devicename, &l_sDefaultUARTConfig) != 0) {
		printf("Error getting attributes!\n");
		goto close_io;
	}

	// setting the UART baud rate from arguments
	printf("Changing the Baud rate.............\n");
	uart_device.baudrate = atoi(argv[2]);
	printf("The baud rate is successfully set at : %u\n",uart_device.baudrate);
	// set the baud rate
	if (cfsetspeed(&l_sUARTConfig, convertIntToSpeedType(uart_device.baudrate)) != 0) {
        	printf("Fail to set the baud rate!\n");
		goto close_io;
	}

	// This is a completely non-blocking read.
	// The call is satisfied immediately directly from the driver's input queue.
	l_sUARTConfig.c_cc[VTIME] = 0;
	l_sUARTConfig.c_cc[VMIN] = 0;

	// activate the settings
	if (tcsetattr(uart_device.devicename, TCSANOW, &l_sUARTConfig) != 0) {
		printf("fail to activate the setting!\n");
		goto close_io;
	}

	// flush before read byte
	if (tcflush(uart_device.devicename, TCIFLUSH) != 0) {
		goto restore_default_config;
	}

	pthread_t write_thread;
	pthread_barrier_init(&barrier_work_main,NULL,2);
	unsigned char Rx_Data[COUNT] = {0, 0, 0, 0, 0};
	int read_count_in_byte = 0;
	double time_elapsed[COUNT];
	double time_elapsed_per_bit[COUNT];
	double time_expected =(double) (8*COUNT) / uart_device.baudrate ;
	double time_difference;

	// create a thread to write bytes
	pthread_create(&write_thread,NULL,&write_bytes,(void*)&uart_device);

	// synchronize main thread and worker thread
	pthread_barrier_wait(&barrier_work_main);
	int count = 0 ;
	printf("-----------Reading-----------\n");
	do {
		count = read(uart_device.devicename, &Rx_Data[read_count_in_byte], 1);
		if (count == 0){
			continue;
		}
		else if (count < 0) {
			//goto clos
			printf("reading failed!\n");
			fprintf(stderr, "Value of errno: %d\n", errno);
			fprintf(stderr, "Error opening file: %s\n", strerror(errno));
			break;
		}
		else {
			gettimeofday(&tval_after[read_count_in_byte], NULL);
			read_count_in_byte += count;
		}

	} while (read_count_in_byte != COUNT );

	for (int i =0 ; i < COUNT; i++ )
	{
		timersub(&tval_after[i], &tval_before, &tval_result[i]);
		time_elapsed[i] = (double)tval_result[i].tv_sec + ((double)tval_result[i].tv_usec/1000000.0f);

		if (i > 0) {
			time_elapsed_per_bit[i] = time_elapsed[i] - time_elapsed[i-1];
		}
		else {
			time_elapsed_per_bit[i] = time_elapsed[i];
		}

		printf("The reading takes %f ms\n", time_elapsed_per_bit[i]*1000);
	}

	time_difference = time_elapsed[COUNT-1] -  time_expected ;
	printf("The difference of the time is %f ms\n", time_difference*1000);
	printf("The expected time is %f ms\n",time_expected*1000);
	printf("The reading time is %f ms\n",time_elapsed[COUNT-1] *1000);

	if ( read_count_in_byte < 0 ) {
			fprintf(stderr, "Value of errno: %d\n", errno);
			fprintf(stderr, "Error opening file: %s\n", strerror(errno));
	}
	else if ( read_count_in_byte != COUNT ) {
			printf("Reading failed!\n");
			err = FAILURE;
	}
	printf("------Finish Reading------\n");
	printf("Expected Bytes received = %d, Actual Bytes received: %d\n", COUNT, read_count_in_byte);
	if (read_count_in_byte == COUNT) {
		err = SUCCESS;
	}

	for (int i = 0; i < read_count_in_byte && read_count_in_byte <= COUNT; i++) {
		if (Tx_Data[i] != Rx_Data[i]) {
			printf("Tx = %c, Rx = %c (Mismatch)\n", Tx_Data[i], Rx_Data[i]);
			err = FAILURE;
		}
		else {
			printf("Tx = %c, Rx = %c\n", Tx_Data[i], Rx_Data[i]);
			err = SUCCESS;
		}
	}

	printf("------------Result-------------\n");
	if ( err == 0) {
		printf("Data match SUCCESS!\n");
	}
	else {
		printf("Data match FAIL!\n");
	}
	printf("--------End of result----------\n");
	pthread_join(write_thread, NULL);
	pthread_barrier_destroy(&barrier_work_main);

	// flush after read byte
	if (tcflush(uart_device.devicename, TCIFLUSH) != 0) {
		err = FAILURE;
	}


restore_default_config:
	if (tcsetattr(uart_device.devicename, TCSANOW, &l_sDefaultUARTConfig) != 0) {
        printf("fail to restore default setting!\n");
		err = FAILURE;
	}

close_io:
	if (close(uart_device.devicename) < 0) {
		printf("io failed to close device");
		err = FAILURE;
	}

end:
	return err;
}
