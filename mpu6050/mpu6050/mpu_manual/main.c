//#define F_CPU 8000000UL

// Code for project

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <math.h>  //include libm

#include "mpu6050/mpu6050.h"

#define UART_BAUD_RATE 38400
#include "uart/uart.h"

/*
#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 0	//changed from 1 because trying to debug

	filters roll pitch and yaw using complementary filter 

void applyCompFilter(double * filteredAngle, double accelAngle, double gyroAngle, double alpha){
	*filteredAngle = alpha * gyroAngle + (1-alpha) * accelAngle;
}
#endif
*/
#if MPU6050_GETATTITUDE == 0

//for changing printing between raw converted data or angles from integrator
#define SERIALPRINTMODE 0		//0 for printing raw converted data
								//1 for printing angles from integrator
// for changing between tests for filtering and integrating obtained data	
#define DATAPROCESSINGTEST 0	//0 for no tests, only run integrator
								//1 for test 1
								//2 for test 2
								//3 for test 3

//offset used for calibrating to zero
#define ACCEL_X_OFFSET 0.0206f
#define ACCEL_Y_OFFSET 0.0216f
#define ACCEL_Z_OFFSET -0.0870f
#define GYRO_X_OFFSET 2.5316f	//2.5610f	these offsets give negative drift
#define GYRO_Y_OFFSET 9.7539f	//9.7561f
#define GYRO_Z_OFFSET 12.3735f	//12.3781f

/*
	calculates the roll pitch and yaw based on acceleration data
*/
void anglesFromAccel(double * rollA, double * pitchA, double ax, double ay, double az){
	*rollA = atan2f(ay, sqrt(square(ax) + square(az)));
	*pitchA = atan2f(ax, sqrt(square(ay) + square(az)));
	//theta = atan2f(az/(sqrt(square(ax) + square(ay))));
}

/*
	calculates integration using runge-kutta integrator
*/
void rk_integrator(double* angle, double dps, double* prev_dps){
	double dps_1 = *prev_dps;
	double dps_2 = *(prev_dps + 1);
	double dps_3 = *(prev_dps + 2);
	*angle = *angle + (dps_3 + 2 * dps_2 + 2 * dps_1 + dps) / 6;
	// updating previous angular velocity array
	*(prev_dps + 2) = dps_2;
	*(prev_dps + 1) = dps_1;
	*prev_dps = dps;
}

/*
	applies offset calibration values to raw converted data
*/
void applyOffset(double* axg, double* ayg, double* azg, double* gxds, double* gyds, double* gzds){
	*axg = *axg - ACCEL_X_OFFSET;
	*ayg = *ayg - ACCEL_Y_OFFSET;
	*azg = *azg - ACCEL_Z_OFFSET;
	*gxds = *gxds - GYRO_X_OFFSET;
	*gyds = *gyds - GYRO_Y_OFFSET;
	*gzds = *gzds - GYRO_Z_OFFSET;	
}

/*
	filters roll pitch and yaw using complementary filter 
*/
void applyCompFilter(double * filteredAngle, double accelAngle, double gyroAngle, double alpha){
	*filteredAngle = alpha * gyroAngle + (1-alpha) * accelAngle;
}

#endif

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
	/*//offset used for calibrating to zero
	double axg_offset = 0.0206;
	double ayg_offset = 0.0216;
	double azg_offset = -0.0870;
	double gxds_offset = 2.5316;
	double gyds_offset = 9.7539;
	double gzds_offset = 12.3735;*/
	//for runge-kutta integrator
	double angleX = 0.0;	//roll
	double angleY = 0.0;	//pitch
	double angleZ = 0.0;	//yaw
	double prev_Xdps[3] = {0.0, 0.0, 0.0};	//holds last three gyro values for 
	double prev_Ydps[3] = {0.0, 0.0, 0.0};	// runge-kutta integrator
	double prev_Zdps[3] = {0.0, 0.0, 0.0};
	double outAngleX = 0.0;		//angles that are outputted to serial port
	double outAngleY = 0.0;
	double outAngleZ = 0.0;
	//used for complementary filter
	double tau = 1.0;		//desired time constant
	double dt = .005;		//based on sampling frequency (200Hz)
	double alpha = tau / (tau + dt);
	//double yawFilt = 0.0;
	double rollAccel = 0.0;	//angles calculated from acceleration data
	double pitchAccel = 0.0;
	#endif
	
	#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2

	double qw = 1.0f;
	double qx = 0.0f;
	double qy = 0.0f;
	double qz = 0.0f;
	double roll = 0.0f;		//around x axis
	double pitch = 0.0f;	//around y axis
	double yaw = 0.0f;		//around z axis
	double roll_d = 0.0f;		//used to convert to degrees
	double pitch_d = 0.0f;
	double yaw_d = 0.0f;
	
	//used for complementary filter
	double tau = 1.0;		//desired time constant
	double dt = .005;		//based on sampling frequency (200Hz)
	double alpha = tau / (tau + dt);
	double rollFilt = 0.0;	//filtered angles
	double pitchFilt = 0.0;
	//double yawFilt = 0.0;
	double rollAccel = 0.0;	//angles calculated from acceleration data
	double pitchAccel = 0.0;
	//double yawAccel = 0,0;
	/*double axg = 0;
	double ayg = 0;
	double azg = 0;
	double gxds = 0;
	double gyds = 0;
	double gzds = 0;
	*/
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
	
/*
 *	Main Loop 
 */	
	for(;;) {
		#if MPU6050_GETATTITUDE == 0
		mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		applyOffset(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		/*rk_integrator(&angleX, gxds, prev_Xdps);
		rk_integrator(&angleY, gyds, prev_Ydps);
		rk_integrator(&angleZ, gzds, prev_Zdps);*/
		
		//No tests are run
		#if DATAPROCESSINGTEST == 0
			rk_integrator(&angleX, gxds, prev_Xdps);
			rk_integrator(&angleY, gyds, prev_Ydps);
			rk_integrator(&angleZ, gzds, prev_Zdps);
			outAngleX = angleX;
			outAngleY = angleY;
			outAngleZ = angleZ;
		#endif
		
		
		//Test 1:integrate then apply fusion filter
		#if DATAPROCESSINGTEST == 1
			rk_integrator(&angleX, gxds, prev_Xdps);
			rk_integrator(&angleY, gyds, prev_Ydps);
			rk_integrator(&angleZ, gzds, prev_Zdps);
			
			anglesFromAccel(&rollAccel, &pitchAccel, axg, ayg, azg);
			applyCompFilter(&outAngleX, rollAccel, angleX, alpha);
			applyCompFilter(&outAngleY, pitchAccel, angleY, alpha);
			outAngleZ = angleZ; 
		#endif
		//Test 2: integrate then apply low pass filter to gyro angle then fusion filter
		#if DATAPROCESSINGTEST == 2
			
		#endif
		
		//Test 3: apply low pass filter to gyro data, integrate, then fusion filter 
		#if DATAPROCESSINGTEST == 3
		#endif
		
		#endif
		
		#if MPU6050_GETATTITUDE == 1
		mpu6050_getQuaternion(&qw, &qx, &qy, &qz);
		//mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);
		//uart_puts("A\r\n");
		//uart_puts("B\r\n");
		mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
		//mpu6050_getAnglesFromAccel(&rollAccel, &pitchAccel);
		
		//mpu6050_getConvData(&axg, &ayg, &azg, &gxds, &gyds, &gzds);
		//anglesFromAccel(&rollAccel, &pitchAccel, axg, ayg, azg);
		//mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
		//applyCompFilter(&rollFilt, rollAccel, roll, alpha);
		//applyCompFilter(&pitchFilt, pitchAccel, pitch, alpha);
		_delay_ms(10);
		#endif
		
		#if MPU6050_GETATTITUDE == 2
		if(mpu6050_getQuaternionWait(&qw, &qx, &qy, &qz)) {
			mpu6050_getRollPitchYaw(qw, qx, qy, qz, &roll, &pitch, &yaw);
		}
		_delay_ms(10);
		#endif

/*
 *	Serial Printing via UART
*/
		#if MPU6050_GETATTITUDE == 0
		char itmp[10];
		/*
		uart_puts("Printing from getRawData \r\n");
		ltoa(ax, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(ay, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(az, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gx, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gy, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		ltoa(gz, itmp, 10); uart_putc(' '); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");
*/
		
		//uart_puts("Printing from getConvData\r\n");
		#if SERIALPRINTMODE == 0
			dtostrf(axg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			dtostrf(ayg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			dtostrf(azg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			dtostrf(gxds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			dtostrf(gyds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			dtostrf(gzds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		#endif
		/*uart_puts("\r\n");
		
	
		uart_puts("X prev_dps array:");
		dtostrf(*prev_Xdps, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(*(prev_Xdps+1), 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(*(prev_Xdps+2), 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");
			*/
		//uart_puts("Angles:");
		#if SERIALPRINTMODE == 1
			dtostrf(outAngleX, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			dtostrf(outAngleY, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			dtostrf(outAngleZ, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			
			/*//temporarily here for testing 
			dtostrf(axg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			dtostrf(ayg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			dtostrf(azg, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			dtostrf(gxds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			dtostrf(gyds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
			dtostrf(gzds, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');*/
			uart_puts("\r\n");
		#endif
		
		uart_puts("\r\n");
		_delay_ms(10);	//originally 1000
		#endif
		
		#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2
		
		//change values to degrees
		roll_d = roll*180/3.1415;
		pitch_d = pitch*180/3.1415;
		yaw_d = yaw*180/3.1415;
		
		char itmp[10];
		//dtostrf(n, 3, 5, itmp); uart_puts(itmp); uart_puts("\r\n");
		dtostrf(roll_d, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(pitch_d, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		dtostrf(yaw_d, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
		uart_puts("\r\n");
		
		uart_puts("\r\n");
		
		_delay_ms(100);
		#endif
		
	}

}
