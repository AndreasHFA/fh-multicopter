#ifndef _IMU_ALGORITHM_H_ 
#define _IMU_ALGORITHM_H_ 

//#define GRAVITY 55

struct IMU_values{
	
	/*dt-time*/	
	float dt;

	/*DCM-Matrix as 3x3 float arrays*/
	float dcm_xe_line[3];
	float dcm_ye_line[3];
	float dcm_ze_line[3];
	
	/*velocity-vector here the gyro values should be placed*/
	float velocity_vector[3];

	float accel_vector[3];
	float mag_vector[3];

	float kp;	
	float error_rollpitch[3];
	float error_yaw;

	float kp_vector_ACC[3];
	float kp_vector_MAG[3];
};

struct EULER_angles{

	float roll;
	float pitch;
	float yaw;

};

struct XYZ_angles
{
	float x;
	float y;
	float z;
};

void IMU_init(struct IMU_values *IMU, float dt);
void IMU_init_drift_correction(struct IMU_values *IMU, float kp);
void IMU_update(struct IMU_values *IMU);
void IMU_normalize(struct IMU_values *IMU);
void IMU_ortho_adjust(struct IMU_values *IMU);
void IMU_drift_correction_ACC(struct IMU_values *IMU);
void IMU_drift_correction_MAG(struct IMU_values *IMU);
void IMU_DCM_to_Euler_deg(struct IMU_values* IMU, struct EULER_angles *euler);
void IMU_Euler_to_DCM(struct IMU_values* IMU, struct EULER_angles *euler);
void IMU_DCM_to_XYZ(struct IMU_values* IMU, struct XYZ_angles *XYZ);
 
#endif // #define _IMU_ALGORITHM_H_
