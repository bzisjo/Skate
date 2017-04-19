/*
 * main_program.c
 *
 * Created: 4/17/2017 7:22:12 PM
 * Author : Jon
 */ 

#include <stdlib.h>
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

volatile uint8_t escape;
volatile uint8_t state;

static uint8_t read_line(char* buffer, uint8_t buffer_length);
static uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry);
static struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name);

ISR(PCINT1_vect)
{
	if(bit_is_clear(PINC, PCINT11))
	{
		_delay_ms(10);		//primitive debouncing
		switch(state)
		{
			case 1: ;
				state = 2;
				break;
			case 2: ;
				if(state == 2 && escape == 0)
				{
					escape = 1;
				}
				break;
		}
	}
}

int main(void)
{
	//Init inputs
	//DDRC &= ~(1 << PORTC3);  //set input mode for button A (start/state switch)

	//Init interrupts
	PCICR |= (1 << PCIE2);		//Set port-change interrupt to port C
	PCMSK1 |= (1 << PCINT11);	//Set port-change mask to PCINT11 (PC3)

	//Init UART
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 

	//Init interrupt
	sei();
	
	//Init MPU6050 & variables
	//int16_t ax = 0;
	//int16_t ay = 0;
	//int16_t az = 0;
	//int16_t gx = 0;
	//int16_t gy = 0;
	//int16_t gz = 0;
	double axg = 0;
	double ayg = 0;
	double azg = 0;
	double gxds = 0;
	double gyds = 0;
	double gzds = 0;
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
		//TODO: LED error pattern 2, opening file partition fail
	}
	//Open root directory
	struct fat_dir_entry_struct directory;
	fat_get_dir_entry_of_path(fs, "/", &directory);
	struct fat_dir_struct* dd = fat_open_dir(fs, &directory);
	if(!dd)
	{
		//TODO: LED error pattern 2, opening root directory failed
	}
	


	//Init state
	state = 1;

    /* Replace with your application code */
    while (1) 
    {
		switch(state)
		{
			//idle
			case 1: ;
				//Wait for button A
				break;
			case 2: ;
				//Data collection
				char file1[] = "masterdata.txt";
				struct fat_dir_entry_struct file_entry;
				if(!fat_create_file(dd, file1, &file_entry)){
					//TODO: LED error pattern 2, create error
				}
				struct fat_file_struct* fd = open_file_in_dir(fs, dd, file1);
				if(!fd)
				{
					//TODO: LED error pattern 2, open error
				}
				int32_t offset = 0;
				if(!fat_seek_file(fd, &offset, FAT_SEEK_END))
				{
					//TODO: LED error pattern 2, fat seek error
				}
				escape = 0;
				while(escape == 0)
				{
					//TODO: Do something to check timmer
					char itmp[10];
					char space = ' ';
					char nl = '\n';
					mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
					dtostrf(axg, 3, 5, itmp); 
					fat_write_file(fd, (uint8_t*) itmp, 10);
					fat_write_file(fd, (uint8_t*) &space, 1);
					dtostrf(ayg, 3, 5, itmp);
					fat_write_file(fd, (uint8_t*) itmp, 10);
					fat_write_file(fd, (uint8_t*) &space, 1);
					dtostrf(azg, 3, 5, itmp);
					fat_write_file(fd, (uint8_t*) itmp, 10);
					fat_write_file(fd, (uint8_t*) &space, 1);
					dtostrf(gxds, 3, 5, itmp);
					fat_write_file(fd, (uint8_t*) itmp, 10);
					fat_write_file(fd, (uint8_t*) &space, 1);
					//dtostrf(gyds, 3, 5, itmp);
					//fat_write_file(fd, (uint8_t*) itmp, 10);
					//fat_write_file(fd, (uint8_t*) &space, 1);
					dtostrf(gzds, 3, 5, itmp);
					fat_write_file(fd, (uint8_t*) itmp, 10);
					fat_write_file(fd, (uint8_t*) &space, 1);

					//TODO: Add lines to read from FSR
					
					fat_write_file(fd, (uint8_t*) &nl, 1);
					
				}
				sd_raw_sync();
				fat_close_file(fd);
				state = 1;
				
				//TODO: after the interrupt, some of the following must be called:
					//sd_raw_sync();
					//fat_close_file(fd);
					//fat_close_dir(dd);
					//fat_close(fs);
					//partition_close(partition);
				break;
		}
		
    }
	return 1;
}



//uSD functions
uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry)
{
	while(fat_read_dir(dd, dir_entry))
	{
		if(strcmp(dir_entry->long_name, name) == 0)
		{
			fat_reset_dir(dd);
			return 1;
		}
	}

	return 0;
}

struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name)
{
	struct fat_dir_entry_struct file_entry;
	if(!find_file_in_dir(fs, dd, name, &file_entry))
	return 0;

	return fat_open_file(fs, &file_entry);
}
