#include <string.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "fat.h"
#include "fat_config.h"
#include "partition.h"
#include "sd_raw.h"
#include "sd_raw_config.h"
#include "uart.h"

#define DEBUG 1

#define UART_BAUD_RATE 9600

static uint8_t read_line(char* buffer, uint8_t buffer_length);
static uint32_t strtolong(const char* str);
static uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry);
static struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name); 
static uint8_t print_disk_info(const struct fat_fs_struct* fs);


int main()
{
    set_sleep_mode(SLEEP_MODE_IDLE);
	//uart_init();
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 
	sei();

	 while(1)
    {
		uart_puts_p(PSTR("going into init\n"));
        /* setup sd card slot */
        if(!sd_raw_init())
        {
        	uart_puts_p(PSTR("MMC/SD initialization failed\n"));
        }
        struct partition_struct* partition = partition_open(sd_raw_read,
                                                            sd_raw_read_interval,
                                                            sd_raw_write,
                                                            sd_raw_write_interval,
                                                            0
                                                           );
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
                uart_puts_p(PSTR("opening partition failed\n"));
            }
        }

        /* open file system */
        struct fat_fs_struct* fs = fat_open(partition);
        if(!fs)
        {
        	uart_puts_p(PSTR("opening filesystem failed\n"));
        }

        /* open root directory */
        struct fat_dir_entry_struct directory;
        fat_get_dir_entry_of_path(fs, "/", &directory);

        struct fat_dir_struct* dd = fat_open_dir(fs, &directory);
        if(!dd)
        {
        	uart_puts_p(PSTR("opening root directory failed\n"));
        }



        /* print some card information as a boot message */
        //print_disk_info(fs);    

       /* char file[] = "test.txt";

        struct fat_file_struct* fd = open_file_in_dir(fs, dd, file);

        uint8_t buffer[512];
        //uint32_t offset = 0;
        intptr_t count;
        while((count = fat_read_file(fd, buffer, sizeof(buffer))) > 0)
        {
            //uart_putdw_hex(offset);
            //uart_putc(':');
            for(intptr_t i = 0; i < count; ++i)
            {

                uart_putc(buffer[i]);
            
                //uart_putc(' ');
                //uart_putc_hex(buffer[i]);
            }
            //uart_putc('\n');
            //offset += 512;
        }
        fat_close_file(fd);*/


        char file1[] = "uSD_Verification.txt";

        struct fat_dir_entry_struct file_entry;
        if(!fat_create_file(dd, file1, &file_entry)){
            uart_puts_p(PSTR("error creating file: "));
            uart_puts(file1);
            uart_putc('\n');
        }

		char file2[] = "uSD_Verification.txt";
		//char* offset_value = file2;

        //while(*offset_value != ' ' && *offset_value != '\0')
                            //++offset_value;

        /*if(*offset_value == ' ')
            *offset_value++ = '\0';
        else
        {
            uart_puts_p(PSTR("error with offset\n"));
            continue;
        }*/

        struct fat_file_struct* fd = open_file_in_dir(fs, dd, file2);

        if(fd == 0)
            uart_puts_p(PSTR("open error\n"));

        int32_t offset2 = 0;

        if(!fat_seek_file(fd, &offset2, FAT_SEEK_END))
        {
            uart_puts_p(PSTR("error seeking on "));
            uart_puts(file2);
            uart_putc('\n');

            fat_close_file(fd);
            continue;
        }
        char testwrite[] = "Device is capable of writing to the uSD connected to the SPI ports on the AVR.";
        //uart_puts(testwrite); uart_putc('\n');
        uart_puts_p(PSTR("writing happens now\n"));
        if(fat_write_file(fd, (uint8_t*) testwrite, (uint8_t)sizeof(testwrite)) != (uint8_t)sizeof(testwrite))
        {
            uart_puts_p(PSTR("error writing to file\n"));
            break;
        }


        fat_close_file(fd);

        if(!sd_raw_sync())
                    uart_puts_p(PSTR("error syncing disk\n"));
		/* close directory */
		fat_close_dir(dd);

		/* close file system */
		fat_close(fs);

		/* close partition */
		partition_close(partition);
		uart_puts_p(PSTR("done\n"));
		while(1){
			continue;
		}


	}

	return 0;
}

uint8_t read_line(char* buffer, uint8_t buffer_length)
{
    memset(buffer, 0, buffer_length);

    uint8_t read_length = 0;
    while(read_length < buffer_length - 1)
    {
        uint8_t c = uart_getc();

        if(c == 0x08 || c == 0x7f)
        {
            if(read_length < 1)
                continue;

            --read_length;
            buffer[read_length] = '\0';

            uart_putc(0x08);
            uart_putc(' ');
            uart_putc(0x08);

            continue;
        }

        uart_putc(c);

        if(c == '\n')
        {
            buffer[read_length] = '\0';
            break;
        }
        else
        {
            buffer[read_length] = c;
            ++read_length;
        }
    }

    return read_length;
}

uint32_t strtolong(const char* str)
{
    uint32_t l = 0;
    while(*str >= '0' && *str <= '9')
        l = l * 10 + (*str++ - '0');

    return l;
}

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

uint8_t print_disk_info(const struct fat_fs_struct* fs)
{
    if(!fs)
        return 0;

    struct sd_raw_info disk_info;
    if(!sd_raw_get_info(&disk_info))
        return 0;

    uart_puts_p(PSTR("manuf:  0x")); uart_putc_hex(disk_info.manufacturer); uart_putc('\n');
    uart_puts_p(PSTR("oem:    ")); uart_puts((char*) disk_info.oem); uart_putc('\n');
    uart_puts_p(PSTR("prod:   ")); uart_puts((char*) disk_info.product); uart_putc('\n');
    uart_puts_p(PSTR("rev:    ")); uart_putc_hex(disk_info.revision); uart_putc('\n');
    uart_puts_p(PSTR("serial: 0x")); uart_putdw_hex(disk_info.serial); uart_putc('\n');
    uart_puts_p(PSTR("date:   ")); uart_putw_dec(disk_info.manufacturing_month); uart_putc('/');
                                   uart_putw_dec(disk_info.manufacturing_year); uart_putc('\n');
    uart_puts_p(PSTR("size:   ")); uart_putdw_dec(disk_info.capacity / 1024 / 1024); uart_puts_p(PSTR("MB\n"));
    uart_puts_p(PSTR("copy:   ")); uart_putw_dec(disk_info.flag_copy); uart_putc('\n');
    uart_puts_p(PSTR("wr.pr.: ")); uart_putw_dec(disk_info.flag_write_protect_temp); uart_putc('/');
                                   uart_putw_dec(disk_info.flag_write_protect); uart_putc('\n');
    uart_puts_p(PSTR("format: ")); uart_putw_dec(disk_info.format); uart_putc('\n');
    uart_puts_p(PSTR("free:   ")); uart_putdw_dec(fat_get_fs_free(fs)); uart_putc('/');
                                   uart_putdw_dec(fat_get_fs_size(fs)); uart_putc('\n');

    return 1;
}

#if FAT_DATETIME_SUPPORT
void get_datetime(uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* hour, uint8_t* min, uint8_t* sec)
{
    *year = 2007;
    *month = 1;
    *day = 1;
    *hour = 0;
    *min = 0;
    *sec = 0;
}
#endif


