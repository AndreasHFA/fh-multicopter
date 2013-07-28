#include <IMU_algorithm.h>
#include <Vector_math.h>
#include <math.h>
#include <stdio.h>


/*Initialization of the main dcm-variables used for the Gyro input*/
void IMU_init(struct IMU_values *IMU, float dt)
{
	IMU->dt = dt;	

	IMU->dcm_xe_line[0] = 1;
	IMU->dcm_xe_line[1] = 0;
	IMU->dcm_xe_line[2] = 0;

	IMU->dcm_ye_line[0] = 0;
	IMU->dcm_ye_line[1] = 1;
	IMU->dcm_ye_line[2] = 0;

	IMU->dcm_ze_line[0] = 0;
	IMU->dcm_ze_line[1] = 0;
	IMU->dcm_ze_line[2] = 1;			

	IMU->velocity_vector[0] = 0;
	IMU->velocity_vector[1] = 0;
	IMU->velocity_vector[2] = 0;
}


/*Initialization of the drift-correction variables - this is mainly solved with a PI-Controller*/
void IMU_init_drift_correction(struct IMU_values *IMU, float kp)
{
	IMU->kp = kp;

	IMU->accel_vector[0] = 0;
	IMU->accel_vector[1] = 0;
	IMU->accel_vector[2] = 0;

	IMU->mag_vector[0] = 0;
	IMU->mag_vector[1] = 0;
	IMU->mag_vector[2] = 0;

	IMU->kp_vector_ACC[0] = 0;
	IMU->kp_vector_ACC[1] = 0;
	IMU->kp_vector_ACC[2] = 0;

	IMU->kp_vector_MAG[0] = 0;
	IMU->kp_vector_MAG[1] = 0;
	IMU->kp_vector_MAG[2] = 0;

	IMU->error_rollpitch[0] = 0;
	IMU->error_rollpitch[1] = 0;
	IMU->error_rollpitch[2] = 0;

	IMU->error_yaw = 0;
}


/*Update of the dcm-matrix*/
void IMU_update(struct IMU_values *IMU)
{
	float result[3];

	/*dt-factor + addition of omega_vector, which holds the PI-Values*/
	IMU->velocity_vector[0] = (IMU->velocity_vector[0]*IMU->dt) - (IMU->kp_vector_ACC[0]) - (IMU->kp_vector_MAG[0]);
	IMU->velocity_vector[1] = (IMU->velocity_vector[1]*IMU->dt) - (IMU->kp_vector_ACC[1]) - (IMU->kp_vector_MAG[1]);
	IMU->velocity_vector[2] = (IMU->velocity_vector[2]*IMU->dt) - (IMU->kp_vector_ACC[2]) - (IMU->kp_vector_MAG[2]);

	/*Update of the first dcm-line (xe)*/
	VectorCrossProduct(result, IMU->dcm_xe_line, IMU->velocity_vector);
	VectorAdd(IMU->dcm_xe_line, IMU->dcm_xe_line, result);

	/*Update of the second dcm-line(ye)*/
	VectorCrossProduct(result, IMU->dcm_ye_line, IMU->velocity_vector);
	VectorAdd(IMU->dcm_ye_line, IMU->dcm_ye_line, result);

	/*Update of the second dcm-line(ze)*/
	VectorCrossProduct(result, IMU->dcm_ze_line, IMU->velocity_vector);
	VectorAdd(IMU->dcm_ze_line, IMU->dcm_ze_line, result);
}


/*Normalization of the vectors*/
void IMU_normalize(struct IMU_values *IMU)
{
	float normfactor;
	normfactor = sqrtf(IMU->dcm_xe_line[0]*IMU->dcm_xe_line[0] + IMU->dcm_xe_line[1]*IMU->dcm_xe_line[1] + IMU->dcm_xe_line[2]*IMU->dcm_xe_line[2]);
	IMU->dcm_xe_line[0] = IMU->dcm_xe_line[0]/normfactor;	
	IMU->dcm_xe_line[1] = IMU->dcm_xe_line[1]/normfactor;	
	IMU->dcm_xe_line[2] = IMU->dcm_xe_line[2]/normfactor;	

	normfactor = sqrtf(IMU->dcm_ye_line[0]*IMU->dcm_ye_line[0] + IMU->dcm_ye_line[1]*IMU->dcm_ye_line[1] + IMU->dcm_ye_line[2]*IMU->dcm_ye_line[2]);
	IMU->dcm_ye_line[0] = IMU->dcm_ye_line[0]/normfactor;	
	IMU->dcm_ye_line[1] = IMU->dcm_ye_line[1]/normfactor;	
	IMU->dcm_ye_line[2] = IMU->dcm_ye_line[2]/normfactor;	

	normfactor = sqrtf(IMU->dcm_ze_line[0]*IMU->dcm_ze_line[0] + IMU->dcm_ze_line[1]*IMU->dcm_ze_line[1] + IMU->dcm_ze_line[2]*IMU->dcm_ze_line[2]);
	IMU->dcm_ze_line[0] = IMU->dcm_ze_line[0]/normfactor;	
	IMU->dcm_ze_line[1] = IMU->dcm_ze_line[1]/normfactor;	
	IMU->dcm_ze_line[2] = IMU->dcm_ze_line[2]/normfactor;		
}

/*Keep the orthogonality*/
void IMU_ortho_adjust(struct IMU_values *IMU)
{
	float ortho_error;

	float result_x[3];
	float result_y[3];

	/*calc the orthogonality error*/	
	ortho_error = -VectorDotProduct(&IMU->dcm_xe_line[0], &IMU->dcm_ye_line[0])*0.5f;
	
	/*add the half of the error to xe and ye*/
	VectorScale(&result_y[0],&IMU->dcm_ye_line[0],ortho_error);
	VectorScale(&result_x[0],&IMU->dcm_xe_line[0],ortho_error);

	VectorAdd(&IMU->dcm_xe_line[0], &IMU->dcm_xe_line[0], &result_y[0]);
	VectorAdd(&IMU->dcm_ye_line[0], &IMU->dcm_ye_line[0], &result_x[0]);
	
	/*the ze-line will be recalculated with the crossproduct*/	
	VectorCrossProduct(&IMU->dcm_ze_line[0],&IMU->dcm_xe_line[0],&IMU->dcm_ye_line[0]);   
}


/*Roll-pitch Drift-Correction with Accelerometer*/
void IMU_drift_correction_ACC(struct IMU_values *IMU)
{
	float normfactor;
	float error_absolute;

	normfactor = sqrtf((IMU->accel_vector[0]*IMU->accel_vector[0]) + (IMU->accel_vector[1]*IMU->accel_vector[1]) + (IMU->accel_vector[2]*IMU->accel_vector[2]));

	if(normfactor != 0)
	{
		IMU->accel_vector[0] = IMU->accel_vector[0]/normfactor;
		IMU->accel_vector[1] = IMU->accel_vector[1]/normfactor;
		IMU->accel_vector[2] = IMU->accel_vector[2]/normfactor;
	}
	
	/*Calc the error between the gyrobased calculations and the vector of the accelerometer, 
	this will be done with the crossproduct of the two vectors*/
	VectorCrossProduct(&IMU->error_rollpitch[0], &IMU->dcm_ze_line[0], &IMU->accel_vector[0]);
	//VectorCrossProduct(&IMU->error_rollpitch[0], &IMU->dcm_ze_line[0], &test[0]);
	
	error_absolute = sqrtf((IMU->error_rollpitch[0]*IMU->error_rollpitch[0]) + (IMU->error_rollpitch[1]*IMU->error_rollpitch[1]) + (IMU->error_rollpitch[2]*IMU->error_rollpitch[2]));

	//printf("%3.1f %3.1f %3.1f %3.1f\n", IMU->accel_vector[0], IMU->accel_vector[1], IMU->accel_vector[2], error_absolute);
	//printf(" %3.1f ", error_absolute);
	/*
	if(error_absolute > 0.6)
	{
		VectorScale(&IMU->kp_vector[0], &IMU->error_rollpitch[0], IMU->kp*2);
	}
	else
	{
		VectorScale(&IMU->kp_vector[0], &IMU->error_rollpitch[0], IMU->kp);
	}
	*/
	VectorScale(&IMU->kp_vector_ACC[0], &IMU->error_rollpitch[0], (IMU->kp+(IMU->kp*error_absolute)));
	//printf("%f\n", (IMU->kp+(IMU->kp*error_absolute)));
}


/*yaw Drift-Correction with Magnetometer*/
void IMU_drift_correction_MAG(struct IMU_values *IMU)
{
	float normfactor;
	//float error_absolute;

	normfactor = sqrtf((IMU->mag_vector[0]*IMU->mag_vector[0]) + (IMU->mag_vector[1]*IMU->mag_vector[1]) + (IMU->mag_vector[2]*IMU->mag_vector[2]));

	if(normfactor != 0)
	{
		IMU->mag_vector[0] = IMU->mag_vector[0]/normfactor;
		IMU->mag_vector[1] = IMU->mag_vector[1]/normfactor;
		IMU->mag_vector[2] = IMU->mag_vector[2]/normfactor;
	}

	/*Calc the error between the gyrobased calculations and the vector of the accelerometer,
	this will be done with the crossproduct of the two vectors*/
	//VectorCrossProduct(&IMU->error_yaw[0], &IMU->dcm_xe_line[0], &IMU->mag_vector[0]);

	IMU->error_yaw = (IMU->dcm_xe_line[0]*IMU->mag_vector[1]) - (IMU->dcm_ye_line[0]*IMU->mag_vector[0]);
	VectorScale(&IMU->kp_vector_MAG[0], &IMU->dcm_ze_line[0], (IMU->kp*IMU->error_yaw));
	//VectorScale(&IMU->kp_vector_MAG[0], &IMU->error_yaw[0], IMU->kp);
	//printf("%f\n", (IMU->kp+(IMU->kp*error_absolute)));
}


/*DCM to EULER Angles*/
void IMU_DCM_to_Euler_deg(struct IMU_values* IMU, struct EULER_angles *euler)
{
	euler->pitch = (asinf(-IMU->dcm_ze_line[0]))*(180/M_PI);
	euler->roll = (atan2f(IMU->dcm_ze_line[1],IMU->dcm_ze_line[2]))*(180/M_PI);
	euler->yaw = (atan2f(IMU->dcm_ye_line[0],IMU->dcm_xe_line[0]))*(180/M_PI);
	//printf("%3.1f  %3.1f  %3.1f\n", roll*(180/M_PI), pitch*(180/M_PI), yaw*(180/M_PI));
}

/*EULER Angles to DCM*/
void IMU_Euler_to_DCM(struct IMU_values* IMU, struct EULER_angles *euler)
{
	/*First line DCM Matrix*/
	IMU->dcm_xe_line[0] = cosf(euler->pitch)*cosf(euler->yaw);
	IMU->dcm_xe_line[1] = (cosf(euler->pitch)*sinf(euler->yaw))+(sinf(euler->roll)*sinf(euler->pitch)*cosf(euler->yaw));
	IMU->dcm_xe_line[2] = (sinf(euler->roll)*sinf(euler->yaw))-(cosf(euler->roll)*sinf(euler->pitch)*cosf(euler->yaw));

	/*Second line DCM Matrix*/
	IMU->dcm_ye_line[0] = -(cosf(euler->pitch)*sinf(euler->yaw));
	IMU->dcm_ye_line[1] = (cosf(euler->roll)*cosf(euler->yaw))-(sinf(euler->roll)*sinf(euler->pitch)*sinf(euler->yaw));
	IMU->dcm_ye_line[2] = (sinf(euler->roll)*cosf(euler->yaw)) + (cosf(euler->roll)*sinf(euler->pitch)*sinf(euler->yaw));

	/*Third line DCM Matrix*/
	IMU->dcm_ze_line[0] = sinf(euler->pitch);
	IMU->dcm_ze_line[1] = -sinf(euler->roll)*cosf(euler->pitch);
	IMU->dcm_ze_line[2] = cosf(euler->roll)*cosf(euler->pitch);
}

void IMU_DCM_to_XYZ(struct IMU_values* IMU, struct XYZ_angles *XYZ)
{
	XYZ->x = asinf(-IMU->dcm_ze_line[0])*(180/M_PI);
	XYZ->y = asinf(IMU->dcm_ze_line[1])*(180/M_PI);
	XYZ->z = (atan2f(IMU->dcm_ye_line[0],IMU->dcm_xe_line[0]))*(180/M_PI);
}
