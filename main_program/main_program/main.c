/*
 * main_program.c
 *
 * Created: 4/17/2017 7:22:12 PM
 * Author : Jon
 */ 

#include <avr/io.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "fat.h"
#include "fat_config.h"
#include "partition.h"
#include "sd_raw.h"
#include "sd_raw_config.h"
#include "uart.h"
#include "mpu6050/mpu6050.h"


#define UART_BAUD_RATE 9600


static uint8_t read_line(char* buffer, uint8_t buffer_length);
static uint32_t strtolong(const char* str);
static uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry);
static struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name);
static uint8_t print_disk_info(const struct fat_fs_struct* fs);

int main(void)
{
	//Init inputs
	DDRC &= ~(1 << PORTC3);  //set input mode for button A (start/state switch)

	//Init UART
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 

	//Init IMU variables
	int16_t ax = 0;
	int16_t ay = 0;
	int16_t az = 0;
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
	double axg = 0;
	double ayg = 0;
	double azg = 0;
	double gxds = 0;
	double gyds = 0;
	double gzds = 0;

	//Init interrupt
	sei();

	//Init MPU6050
	mpu6050_init();
	_delay_ms(50);

	//Init SD
	if(!sd_raw_init())
	{
		//TODO: LED error pattern 1, init failed
	}
	//Open partition
	struct partition_struct* partition = partition_open(sd_raw_read, sd_raw_read_interval, sd_raw_write, sd_raw_write_interval, 0);
	if(!partition)
    {
        /* If the partition did not open, assume the storage device
            * is a "superfloppy", i.e. has no MBR.
            */
        partition = partition_open(sd_raw_read,
                                    sd_raw_read_interval,
                                    sd_raw_write,
                                    sd_raw_write_interval,
                                    -1
                                    );
        if(!partition)
        {
            //TODO: LED error pattern 2, opening partition fail
        }
    }
	//Open file system
	struct fat_fs_struct* fs = fat_open(partition);
	if(!fs)
	{
		//TODO: LED error pattern 3, opening file partition fail
	}
	//Open root directory
	struct fat_dir_entry_struct directory;
	fat_get_dir_entry_of_path(fs, "/", &directory);
	struct fat_dir_struct* dd = fat_open_dir(fs, &directory);
	if(!dd)
	{
		//TODO: LED error pattern 4, opening root directory failed
	}
	


	//Init state
	uint8_t state = 1;

    /* Replace with your application code */
    while (1) 
    {
		switch(state)
		{
			//idle
			case 1:
				//Wait for button A
				break;
			case 2:
				//Data collection
				char file1[] = "masterdata.txt";
				struct fat_dir_entry_struct file_entry;
				if(!fat_create_file(dd, file1, &file_entry)){
					uart_puts_p(PSTR("error creating file: "));
					uart_puts(file1);
					uart_putc('\n');
				}
				struct fat_file_struct* fd = open_file_in_dir(fs, dd, file1);
				if(!fd)
				{
					//TODO: LED pattern 5, open error
				}
				
				while(1)
				{
					mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
					mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);					
				}
				break;
		}
		
    }
}

