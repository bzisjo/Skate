//#define F_CPU 8000000UL

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>  //include libm

#include "mpu6050/mpu6050.h"

#define UART_BAUD_RATE 57600
#include "uart/uart.h"

int main(void) {

	#if MPU6050_GETATTITUDE == 0
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
	#endif

	//init uart
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));

	//init interrupt
	sei();

	//init mpu6050
	mpu6050_init();
	_delay_ms(50);

	uart_puts("Entering main loop\r\n");

	for(;;) {
		#if MPU6050_GETATTITUDE == 0
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		#endif

		#if MPU6050_GETATTITUDE == 0
		char itmp[10];
		ltoa(ax, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(ay, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(az, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gx, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gy, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gz, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");

		dtostrf(axg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(ayg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(azg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gxds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gyds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(gzds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");

		uart_puts("\r\n");

		_delay_ms(1000);
		#endif
	}

}
