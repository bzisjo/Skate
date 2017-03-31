//#define F_CPU 8000000UL

// Code for project

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>  //include libm

#include "mpu6050/mpu6050.h"

#define UART_BAUD_RATE 9600
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
	
	#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2
	//long *ptr = 0;
	/*int16_t ax = 0;	//used because need to get raw data for mahony filter
	int16_t ay = 0;
	int16_t az = 0;
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
	*/
	double qw = 1.0f;
	double qx = 0.0f;
	double qy = 0.0f;
	double qz = 0.0f;
	double roll = 0.0f;
	double pitch = 0.0f;
	double yaw = 0.0f;
	double roll_d = 0.0f;		//used to convert to degrees
	double pitch_d = 0.0f;
	double yaw_d = 0.0f;
	#endif

	//init uart
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));

	//init interrupt
	sei();

	//init mpu6050
	mpu6050_init();
	_delay_ms(50);

	//uart_puts("Entering main loop\r\n");
	
	//init mpu6050 dmp processor
	#if MPU6050_GETATTITUDE == 2
	mpu6050_dmpInitialize();
	mpu6050_dmpEnable();
	_delay_ms(10);
	#endif
	for(;;) {
		//uart_puts("running");
		#if MPU6050_GETATTITUDE == 0
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		#endif
		
		#if MPU6050_GETATTITUDE == 1
		mpu6050_getQuaternion(&qw, &qx, &qy, &qz);
		//mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		//uart_puts("A\r\n");
		//mpu6050_updateQuaternion();	//applies mahony filter to deal with drift?
		//uart_puts("B\r\n");
		mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
		_delay_ms(10);
		#endif
		
		#if MPU6050_GETATTITUDE == 2
		if(mpu6050_getQuaternionWait(&qw, &qx, &qy, &qz)) {
			mpu6050_getRollPitchYaw(qw, qx, qy, qz, &roll, &pitch, &yaw);
		}
		_delay_ms(10);
		#endif

		#if MPU6050_GETATTITUDE == 0
		char itmp[10];
		uart_puts("Printing from getRawData \r\n");
		ltoa(ax, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(ay, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(az, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gx, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gy, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gz, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");

		uart_puts("Printing from getConvData\r\n");
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
		
		#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2
		/*//roll pitch yaw
		ptr = (long *)(&roll);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);
		ptr = (long *)(&pitch);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);
		ptr = (long *)(&yaw);
		uart_putc(*ptr);
		uart_putc(*ptr>>8);
		uart_putc(*ptr>>16);
		uart_putc(*ptr>>24);

		uart_putc('\n');
		*/
		
		//change values to degrees
		roll_d = roll*180/3.1415;
		pitch_d = pitch*180/3.1415;
		yaw_d = yaw*180/3.1415;
		
		char itmp[10];
		dtostrf(roll_d, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(pitch_d, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(yaw_d, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");
		
		uart_puts("\r\n");
		
		_delay_ms(1000);
		#endif
		
	}

}
