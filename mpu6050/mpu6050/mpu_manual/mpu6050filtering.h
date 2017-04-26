#ifndef MPU6050FILTERING_H_
#define MPU6050FILTERING_H_

#include <avr/io.h>

#define PCBOWNERSHIP 0	//Indicates for which pcb the code is for.
						//Each pcb has its own IMU with its own offsets
						// 0	pcb is the master device. Contains the marked IMU
						// 1	pcb is the slave device. Contains the unmarked IMU
#define ALPHA 0.65f		//Alpha coefficient used for the complimentary filter

//offset used for calibrating to zero (median)
#if PCBOWNERSHIP == 0				//Offsets for unmarked IMU
	#define ACCEL_X_OFFSET 0.0222f	
	#define ACCEL_Y_OFFSET 0.0371f	
	#define ACCEL_Z_OFFSET -0.0866f	
	#define GYRO_X_OFFSET 0.0076f;	
	#define GYRO_Y_OFFSET 1.2977f;	
	#define GYRO_Z_OFFSET 1.2519f;	
#endif
#if PCBOWNERSHIP == 1				//Offsets for marked IMU
	#define ACCEL_X_OFFSET 0.0177f;	
	#define ACCEL_Y_OFFSET 0.0438f;
	#define ACCEL_Z_OFFSET -0.0621f;
	#define GYRO_X_OFFSET -2.9008f;
	#define GYRO_Y_OFFSET 0.6870f;
	#define GYRO_Z_OFFSET -0.0534f;
#endif

// functions from the c file
extern void anglesFromAccel(double * rollA, double * pitchA, double ax, double ay, double az);

extern void rk_integrator(double* angle, double dps, double* prev_dps);

extern void applyOffset(double* axg, double* ayg, double* azg, double* gxds, double* gyds, double* gzds);

extern void applyCompFilter(double * filteredAngle, double accelAngle, double gyroAngle);

#endif