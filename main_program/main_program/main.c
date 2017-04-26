/*
 * main_program.c
 *
 * Created: 4/17/2017 7:22:12 PM
 * Author : Also Charles and Qian
 */ 

#include <stdlib.h>
#include <avr/io.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include "fat.h"
#include "fat_config.h"
#include "partition.h"
#include "sd_raw.h"
#include "sd_raw_config.h"
#include "uart.h"
#include "mpu6050/mpu6050.h"


#define UART_BAUD_RATE 9600
#define IMU 0

volatile uint8_t escape;
volatile uint8_t state;
volatile uint8_t error;

//static uint8_t read_line(char* buffer, uint8_t buffer_length);
static uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry);
static struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name);
static inline void init_ADC(void);
static uint8_t FSR_read(int i);

ISR(PCINT2_vect)
{
	_delay_ms(10);
	if(bit_is_clear(PIND, PCINT19))
	{
		state = 1;
		escape = 1;
	}
}

ISR(PCINT1_vect)
{
	
	_delay_ms(10);		//primitive debouncing
	if(bit_is_clear(PINC, PCINT11))
	{
		switch(state)
		{
			case 0: ;
				if(error)
					error = ~error;
				break;
			case 1: ;
				state = 2;
				break;
			case 2: ;
				if(state == 2 && escape == 0)
				{
					escape = 1;
					state = 1;
				}
				break;
		}
	}
}

int main(void)
{
	//*^_^* 
	//timer 1 without interrupt
	//OCRn: set value that you want to count to 01F3 = 499(10)
	//Using 16-bit timer 1, each count up takes 1/16M*1024 = 64us
	//To achieve 31.2Hz, use count top = 499
	//Period = 64u * 500 = 0.032s
 	OCR1A = 0xFFFF;
	//OCR1A = 0x0003;

    // Mode 4, CTC top on OCR1A
    TCCR1B |= (1 << WGM12);
 
    //disbale timer for now by setting CS12 and CS10 to 0 
    TCCR1B &= ~(1 << CS12);
    TCCR1B &= ~(1 << CS10);

    //init ADC
    init_ADC();
	//*^_^*

	//Init inputs
	DDRC &= ~(1 << PORTC3);  //set input mode for button A (start/state switch)
    DDRD &= ~(1 << PORTD3); //set input mode for button B (reset state switch)	
	DDRD |= (1 << PORTD6) | (1 << PORTD7);	//Set LEDs as output

	//Init interrupt
	PCICR |= (1 << PCIE1) | (1 << PCIE2);		//Set port-change interrupt to port C &D
	PCMSK1 |= (1 << PCINT11);	//Set port-change mask to PCINT11 (PC3)
	PCMSK2 |= (1 << PCINT19);   //set port-change mask to PCINT19 (PD3)
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
	#if(IMU)
	double axg = 0;
	double ayg = 0;
	double azg = 0;
	double gxds = 0;
	double gyds = 0;
	double gzds = 0;
	mpu6050_init();
	_delay_ms(50);
	#endif
	

	//Init state
	state = 1;
	//uart_putc('a');
	//Init SD
	if(!sd_raw_init())
	{
		//TODO: LED error pattern 1, init failed
		error = 1;
		state = 0;
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
			error = 1;
			state = 0;
        }
    }
	//Open file system
	struct fat_fs_struct* fs = fat_open(partition);
	if(!fs)
	{
		//TODO: LED error pattern 2, opening file partition fail
		error = 1;
		state = 0;
	}
	//Open root directory
	struct fat_dir_entry_struct directory;
	fat_get_dir_entry_of_path(fs, "/", &directory);
	struct fat_dir_struct* dd = fat_open_dir(fs, &directory);
	if(!dd)
	{
		//TODO: LED error pattern 2, opening root directory failed
		error = 1;
		state = 0;
	}
	


	PORTD |= (1 << PORTD6);		//power LED on
	//uart_putc('b');
    while (1) 
    {
		switch(state)
		{
			//errors
			case 0: ;
				while(error)
				{
					PORTD |= (1 << PORTD7);
					_delay_ms(100);
					PORTD &= ~(1 << PORTD7);
					_delay_ms(100);
				}
				break;
			//idle
			case 1: ;
				//Wait for button A
				while(state == 1)
				{
					PORTD |= (1 << PORTD7);
					_delay_ms(500);
					PORTD &= ~(1 << PORTD7);
					_delay_ms(500);
				}
				break;
			case 2: ;
				//Data collection
				char file1[] = "masterdata.txt";
				struct fat_dir_entry_struct file_entry;
				if(!fat_create_file(dd, file1, &file_entry)){
					//TODO: LED error pattern 2, create error
					error = 1;
					state = 0;
					break;
				}
				struct fat_file_struct* fd = open_file_in_dir(fs, dd, file1);
				if(!fd)
				{
					//TODO: LED error pattern 2, open error
					error = 1;
					state = 0;
					break;
				}
				int32_t offset = 0;
				if(!fat_seek_file(fd, &offset, FAT_SEEK_END))
				{
					//TODO: LED error pattern 2, fat seek error
					error = 1;
					state = 0;
					break;
				}
				escape = 0;
				uint16_t index = 1;
				uint16_t Vbattery = 9;
				char itmp[10];
				char fatbuf[70];
				uint8_t fsr[3];
				while(escape == 0)
				{   
					//set prescaler for Timer 1 to clk/1024
					TCCR1B |= (1 << CS12) | (1 << CS10);
					//start timer count	
					
					//read from FSR and write the data 

					fsr[0] = FSR_read(0); 
					fsr[1] = FSR_read(0);


					itoa(fsr[0],itmp,10);
					strcpy(fatbuf, itmp);			//watch out this is special.
					strcat(fatbuf, " ");
					itoa(fsr[1],itmp,10);
					strcat(fatbuf, itmp);
					strcat(fatbuf, " ");

					#if(IMU)
					mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
					dtostrf(axg, 3, 5, itmp); 
					strcat(fatbuf, itmp);
					strcat(fatbuf, " ");
					dtostrf(ayg, 3, 5, itmp);
					strcat(fatbuf, itmp);
					strcat(fatbuf, " ");
					dtostrf(azg, 3, 5, itmp);
					strcat(fatbuf, itmp);
					strcat(fatbuf, " ");
					dtostrf(gxds, 3, 5, itmp);
					strcat(fatbuf, itmp);
					strcat(fatbuf, " ");
					//dtostrf(gyds, 3, 5, itmp);
					dtostrf(gzds, 3, 5, itmp);
					strcat(fatbuf, itmp);
					#endif

					strcat(fatbuf, "\r\n");
					fat_write_file(fd, (uint8_t*) fatbuf, strlen(fatbuf));
					
					//approximately check battery volrage every 16s
					if(index % 500 == 0)
					{   //DIGITAL PIN TO TURN ON NMOS
						//enables PD4 as digital output and set PD4 HIGH
						DDRD |= (1 << PD4);
						PORTD |= (1 << PD4);
						fsr[3] = FSR_read(1);
						//let's do math here
						//fsr3 is a 8-bit value ranging from 0-255, correction factor k = (0.3943/0.3532)
						//in terms of voltage, Vsense = Vbattery * (6.05/(6.05+11.96)) * k = (fsr[3]/255) * 3.3
						Vbattery = (fsr[3]/255) * 3.3 / ((6.05/(6.05+11.96)) * (0.3943/0.3532));
						if(Vbattery < 5)
						{
						escape = 1;
						state = 0;	
						error = 1; //*this is baaaaaaaaaaaaaad
						}
						DDRD &= ~(1 << PD4);
					}
					//itoa((TIFR0 & (1 << TOV0) ) > 0,itmp,10);
					//uart_putc(*itmp);4
					index ++;
					while( (TIFR0 & (1 << TOV0) ) > 0) //wait for timer overflow event
					{
					}
					
				}
				sd_raw_sync();
				fat_close_file(fd);
				
				//TODO(???): after the interrupt, some of the following must be called:
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

//*^_^*
//FSR functions
static inline void init_ADC()
{
	/*
	ADMUX - ADC Multiplexer Selection Register
	
	bit          7           6          5         4        3         2          1          0
	name       REFS1       REFS0      ADLAR       -       MUX3      MUX2       MUX1       MUX0
	set to       0           1          1         0        0         1          0          1
	
	REFS1 = 0    use AVCC for reference voltage
	REFS0 = 1
	
	ADLAR = 1    left justify ADC result in ADCH/ADCL
	
	bit 4 = 0
	
	MUX3 = 0     start with using ADC0 (pin 23) for input
	MUX2 = 0
	MUX1 = 0
	MUX0 = 0
	*/
	ADMUX = 0b01100000;

		/*
	ADCSRA - ADC Control and Status Register A
	
	bit          7           6            5          4          3            2           1           0
	name        ADEN        ADSC        ADATE       ADIF       ADIE        ADPS2       ADPS1       ADPS0
	set to       1           0            0          0          0            0           1           1
	
	ADEN = 1     enable ADC
	ADSC = 0     don't start ADC yet
	ADATE = 0    don't enable ADC auto trigger (i.e. use single conversion mode)
	ADIF = 0     don't set ADC interrupt flag
	ADIE = 0     don't set ADC interrupt enable
	
	ADPS2 = 1
	ADPS1 = 1    16 MHz clock / 128 = 125 kHz ADC clock
	ADPS0 = 1
	*/
	ADCSRA = 0b10000111;
	
	/*
	ADCSRB - ADC Control and Status Register B
	
	bit         7           6           5           4           3         2           1           0
	name        -          ACME         -           -           -       ADTS2       ADTS1       ADTS0
	set to      0           0           0           0           0         0           0           0
	
	bit 7 = 0
	ACME = 0     don't enable analog comparator multiplexer
	bit 5 = 0
	bit 4 = 0
	bit 3 = 0
	ADTS2 = 0
	ADTS1 = 0    register ADCSRA bit ADATE set to 0 so these bits have no effect
	ADTS0 = 0
	*/
	ADCSRB = 0b00000000;
}

//reading values from analog pins
uint8_t FSR_read(int i)
{	
	//read pin25 value as voltage input
	if(i == 1)
	{
		ADMUX &= ~(1 << PORTC0);//clear PC0 bit
		ADMUX |= (1 << PORTC1); //enable pin25
	}
	else
	{
		ADMUX &= ~(1 << PORTC1);//clear PC1 bit, disable pin25
		ADMUX ^= (1 << PORTC0); //toggle between pin23 and 24	
	}
	// start ADC conversion
	ADCSRA |= (1 << ADSC);		
	// wait here until the chip clears the ADSC bit for us, which means the ADC is complete			
	while(bit_is_set(ADCSRA, ADSC)) {}	
	// toggle ADMUX PC0 bit to switch between pin23 and pin24				
				
	return ADCH;
}
//*^_^*

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
