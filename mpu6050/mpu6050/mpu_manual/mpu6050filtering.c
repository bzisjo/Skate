/*
 *	Source file for angle filtering used along the mpu6050 library.
 *	Includes:
 *	IMU specific offsets
 *	Kutta-Runge Integrator
 *	Angle Acquisition based on accelerometer data
 *	Complimentary Filter
 */

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <math.h>  //include libm

#include "mpu6050filtering.h"

/*
	Calculates the roll and pitch based on acceleration data
	Outputs calculated angle in degrees (rollA and pitchA)
*/
void anglesFromAccel(double * rollA, double * pitchA, double ax, double ay, double az){
	*rollA = atan2f(ay, sqrt(square(ax) + square(az)))*180/3.1416;
	*pitchA = - atan2f(ax, sqrt(square(ay) + square(az)))*180/3.1416;
	//theta = atan2f(az/(sqrt(square(ax) + square(ay))));
}

/*
	Calculates integration using runge-kutta integrator method.
	Function needs an input of a buffer of type double that is three values in size.
	The buffer is used to hold previous values due to the nature of the rk integrator
	Reference of rk integrator can be found here: http://tom.pycke.be/mav/70/gyroscope-to-roll-pitch-and-yaw
	Equation of rk integrator: integration(i) = integration(i-1) + 1?6 ( vali-3 + 2 vali-2 + 2 vali-1 + vali)
*/
void rk_integrator(double* angle, double dps, double* prev_dps){
	double dps_1 = *prev_dps;
	double dps_2 = *(prev_dps + 1);
	double dps_3 = *(prev_dps + 2);
	double input = dps/60;		//runs around 60 Hz about
	*angle = *angle + (dps_3 + 2 * dps_2 + 2 * dps_1 + input) / 6;
	//check to make sure not greater than -360 and 360
	if ((*angle > 360) || (*angle < -360)){
		*angle = 0.0;
	}
	// updating previous angular velocity array
	*(prev_dps + 2) = dps_2;
	*(prev_dps + 1) = dps_1;
	*prev_dps = input;
}

/*
	Applies offset calibration values to raw converted data
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
	Applies complimentary filter to an angle 
*/
void applyCompFilter(double * filteredAngle, double accelAngle, double gyroAngle){
	*filteredAngle = (1-ALPHA) * gyroAngle + ALPHA * accelAngle;
}