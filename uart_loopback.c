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

// the number of bytes to write
#define COUNT 100
#define SUCCESS 0
#define FAILURE -1
#define ARRAY_SIZE 25

extern int errno ;

pthread_barrier_t barrier_work_main;
int baudrate_ls[ARRAY_SIZE] = {50,75,110,134,150,200,300,600,1200,2400,4800,9600,19200,38400,
		       57600,115200,230400,460800,921600,1000000,1152000,2000000,
		       2500000,3000000,4000000};
const unsigned char TX_DATA[COUNT] = "Hello World from StarFive Technology International";
struct timeval tval_before, tval_after[COUNT], tval_result[COUNT];

// A struct that passes parameters into a function that is used
// by thread.
struct Device {
	int devicename;
	unsigned int baudrate;
}uart_device;

void *write_bytes(void* device)
{
	struct Device *device_ptr = ( struct Device*) device;
	int fd = device_ptr->devicename;
	unsigned int baudrate = device_ptr->baudrate;

	// synchronize main thread and worker thread
	if (write(fd, TX_DATA, COUNT) != COUNT) {
		printf("error in write\n");
	}

	// add a barrier here
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
	int min_baudrate;
	int max_baudrate;

	// make sure the correct input arguments number
	if (argc != 4) {
		printf("Please input three arguments!\n");
		printf("Example: ./uart_loopback /dev/ttyS3 9600 115200\n");
		goto end;
	}

	// copy the name of the device
	strcpy(sDevice,argv[1]);
	min_baudrate = atoi(argv[2]);
	max_baudrate = atoi(argv[3]);

	int min_index = -1;
	int max_index = -1;

	for (int i = 0; i < ARRAY_SIZE; i++ ) {
		if (baudrate_ls[i] == min_baudrate ) {
			min_index = i;
			printf("min index is %d\n", min_index);
		}
		else if ( baudrate_ls[i] == max_baudrate ) {
			max_index = i;
			printf("max index is %d\n", max_index);
		}

		if ( max_index <= min_index || max_index <= 0 || min_index <= 0 ) {
			printf("Error in baud rate finding!\n");
			err = FAILURE;
			goto end;
		}
	}
	// open the uart device for reading and writing || not control tty because
	// we don't want to get killed if linenoise sends CTRL-C || open the device
	// in non-blocking mode
	uart_device.devicename = open(sDevice, O_RDWR | O_NOCTTY | O_NDELAY);

	// make sure the uart device is correctly opened
	if (uart_device.devicename < 0) {
                printf("io failed to open device\n");
		goto end;
	}

	printf("Executing UART LOOPBACK TEST for device %s\n", sDevice);
	struct termios l_uart_config, l_ori_uart_config;
	for (int index = min_index; index < max_index; index ++ ) {

		uart_device.baudrate = baudrate_ls[index];

		if ( ( tcgetattr(uart_device.devicename, &l_uart_config)) != 0
		|| ( tcgetattr(uart_device.devicename, &l_ori_uart_config ) ) != 0 ) {
			printf("Error getting attributes!\n");
			goto close_io;
		}

		// setting the UART baud rate from arguments
		printf("Changing the Baud rate.............\n");
		printf("The baud rate is successfully set at : %u\n",uart_device.baudrate);

		int ispeed = cfsetispeed(&l_uart_config, convertIntToSpeedType(uart_device.baudrate));
		int ospeed = cfsetospeed(&l_uart_config, convertIntToSpeedType(uart_device.baudrate));

		// set the baud rate
		if ( ispeed != 0 || ospeed != 0 ) {
			printf("Fail to set the baud rate!\n");
			goto close_io;
		}

		// the configuration for input/output/control/local mode
		// clear the settings for the following flags
		l_uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
			| INLCR | IGNCR | ICRNL | IXON);
		l_uart_config.c_oflag &= ~OPOST;
		l_uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		l_uart_config.c_cflag &= ~(CSIZE | PARENB);
		l_uart_config.c_cflag |= CS8 | CREAD | CLOCAL;
		l_uart_config.c_cc[VTIME] = 0;
		l_uart_config.c_cc[VMIN] = 0;

		// activate the settings
		if (tcsetattr(uart_device.devicename, TCSANOW, &l_uart_config) != 0) {
			printf("fail to activate the setting!\n");
			goto restore_default_config;
		}

		if ( cfgetispeed(&l_uart_config) != convertIntToSpeedType(uart_device.baudrate) ) {
			printf("error get speed!\n");
		}
		// flush before reading byte
		if (tcflush(uart_device.devicename, TCIOFLUSH) != 0) {
			goto restore_default_config;
		}

		// create a thread that writes bytes to tx pin
		pthread_t write_thread;
		// init a thread barrier with 2 counts
		pthread_barrier_init(&barrier_work_main,NULL,2);

		// the rx data with the same size as tx data
		unsigned char Rx_Data[COUNT];

		// the variable that stores the number of bytes read
		int read_count_in_byte = 0;

		// create a thread to write bytes
		pthread_create(&write_thread,NULL,&write_bytes,(void*)&uart_device);

		// synchronize main thread and worker thread
		// first barrier count here
		pthread_barrier_wait(&barrier_work_main);

		int count = 0 ;
		double time_elapsed[COUNT];
		double time_elapsed_per_bit[COUNT];
		double time_expected =(double) (8*COUNT) / uart_device.baudrate ;
		int divisor = (100000000) / (16 * uart_device.baudrate);
		int real_baudrate = ( 100000000 ) / (16 * divisor);
		// printf("The divisor is %d\n", divisor);
		// printf("The real baud rate is %d\n", real_baudrate);
		double time_difference;
		printf("-----------Reading-----------\n");

		do {
			count = read(uart_device.devicename, &Rx_Data[read_count_in_byte], 1);
			if (count == 0){
				// printf("Count = 0\n");
				continue;
			}
			else if (count < 0) {
				//goto close
				printf("reading failed!\n");
				fprintf(stderr, "Value of errno: %d\n", errno);
				fprintf(stderr, "Error opening file: %s\n", strerror(errno));
				err = FAILURE;
				break;
			}
			else {
				if ( read_count_in_byte == 0 ){
					gettimeofday(&tval_before, NULL);
				}
				gettimeofday(&tval_after[read_count_in_byte], NULL);
				read_count_in_byte += count;
			}

		} while (read_count_in_byte != COUNT );

		printf("Reading SUCCESS!\n");
		printf("-----------Result------------\n");

		for (int i =0 ; i < COUNT; i++ )
		{
			timersub(&tval_after[i], &tval_before, &tval_result[i]);
			time_elapsed[i] = (double)tval_result[i].tv_sec + ((double)tval_result[i].tv_usec/1000000.0f);

			if (i > 0) {
				time_elapsed_per_bit[i] = time_elapsed[i] - time_elapsed[i-1];
			} else {
				time_elapsed_per_bit[i] = time_elapsed[i];
			}
			//printf("The reading takes %f ms\n", time_elapsed_per_bit[i]*1000);
		}

		time_difference = time_elapsed[COUNT-1] - time_expected ;
		// printf("The difference of the time is %f ms\n", time_difference*1000);
		printf("The expected time is %f ms\n",time_expected*1000);
		printf("The reading time is %f ms\n",time_elapsed[COUNT-1]*1000);
		for (int i = 0; i < read_count_in_byte && read_count_in_byte <= COUNT; i++) {
			if (TX_DATA[i] != Rx_Data[i]) {
				//printf("Tx = %c, Rx = %c (Mismatch)\n", Tx_Data[i], Rx_Data[i]);
				err = FAILURE;
				break;
			} else {
				// printf("Tx = %c, Rx = %c\n", Tx_Data[i], Rx_Data[i]);
				err = SUCCESS;
			}
		}

		if ( err == SUCCESS) {
			printf("Data match SUCCESS!\n");
		}
		else {
			printf("Data match FAIL!\n");
		}
		printf("-------End of result---------\n");
		pthread_join(write_thread, NULL);
		pthread_barrier_destroy(&barrier_work_main);

		// flush after read byte
		if (tcflush(uart_device.devicename, TCIFLUSH) != 0) {
			err = FAILURE;
		}
	}


restore_default_config:
	printf("Restoring config to default\n");
	if (tcsetattr(uart_device.devicename, TCSANOW, &l_ori_uart_config) != 0) {
		printf("fail to restore default setting!\n");
		err = FAILURE;
	}

close_io:
	printf("Closing IO\n");
	if (close(uart_device.devicename) < 0) {
		printf("io fails to close device");
		err = FAILURE;
	}

end:
	return err;
}
