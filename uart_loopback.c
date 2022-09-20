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

#define COUNT 10
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
		case 921600	: return B921600;
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
	// start with an error return value
	int err = FAILURE;

	// a string that stores the device driver name
	char sDevice[128];
	strcpy(sDevice,argv[1]);
	uart_device.devicename = open(sDevice, O_RDWR | O_NOCTTY | O_NDELAY );

	// make sure the uart device is correctly opened
	if (uart_device.devicename < 0) {
                printf("io failed to close device");
		goto end;
	}

	uart_device.baudrate = atoi(argv[2]);

	if (argc != 3) {
		printf("Please input two arguments!\n");
		printf("Example: ./uart_loopback /dev/ttyS3 9600\n");
		goto end;
	}

	printf("Executing UART LOOPBACK TEST for device %s\n", sDevice);




end:
	return err;
}
