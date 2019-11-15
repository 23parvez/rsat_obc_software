#ifndef ADCS_SENSORDATAPROCS_H_INCLUDED
#define ADCS_SENSORDATAPROCS_H_INCLUDED

///IMU

double w_BODY[3], w_BODYdeg[3], Thta_rawdata[3], B_BODY[3], B_BODYn[3], B_BODYtesla[3], Bsq, w_BODYnorm;
double DelThta_rawdata[3], w_ABC[3], w_BiasCor[3], w_ASC[3], w_AMSC[3],w_RPY[3],w_LPF[3], B_ABC[3];
double B_IMU[3], B_BiasCor[3], B_ASC[3], B_AMSC[3],B_RPY[3],B_LPF[3];
double w_BODY_IMU1[3], w_BODY_IMU2[3], B_BODY_IMU1[3], B_BODY_IMU2[3];
double IMU_prcd_data[6];
double* IMU_prcd_data_ptr;
double IMU_Sen2Bdy[3][3];

unsigned int imu1_db_checksum_obc;
unsigned int imu2_db_checksum_obc;

int B_rawdata[3]; // Magnetic field raw data
int inter_imu_i,inter_imu_j;
double Thta_BODY_IMU1[3], Thta_BODY_IMU2[3];

///16 Cells' output (Main and Redundant)
///Redundant Cells

double SS_Data[16];
int inter_sunsensor_i,inter_sunsensor_j;

///8 selected Sensors' output
double Roll_pos_D, Roll_neg_D, Roll_pos_ND, Roll_neg_ND, Pitch_pos, Pitch_neg, Yaw_pos, Yaw_neg;

double *SS_prcd_data_ptr;
double SB_MAIN[3], SB_RED[3], SS_prcd_data[3];

///6 Active sensors' output after multiplying by constants
double SC1, SC2, SC3, SC4, SC5, SC6, SC7, SC8;

///6 Cells' Imax Factors
double SC1ImaxF, SC2ImaxF, SC3ImaxF, SC4ImaxF, SC5ImaxF, SC6ImaxF;

int TC_SS_Main_Cells_Sel, Panel_Deployment;;

double comb1, comb2, temp11, temp12;

double ele1, az1, ele2; ///Elevation and azimuth angles

double sun_sf[3],S_BODY[3], S_BODYn[3]; ///SunSensor data in sensor frame

double ss2b[3][3];

double Ang_Deviation; ///arccos(dot(S_BODY,[0,-1,0]))

double Roll_ang_err, Yaw_ang_err;
int sun_quadrant;
int f_Sunlit_Presence;

int i_MatEq, j_MatEq;
int Sunlit_presence_timer;

double AngDev_SMtransit_thrsld;
int SunNPP_SMtransit_counter;
int SunNPP_SMtransit_count_limit;
int SunNPP_SMtransit;

double AngDev_SAMtransit_thrsld;
int SunNPP_SAMtransit_counter;
int SunNPP_SAMtransit_count_limit;
int SunNPP_SAMtransit;

//Checksum
unsigned long int ss_main_db_checksum_obc;
unsigned long int ss_redundant_db_checksum_obc;

///BDOT Computation

int BDOT_Counter;
int Det_BDOT_MC_Count;

double BDOT[3],Bpresent[3];
double Bprev[3];

///Det gyro

int GYRO_Counter;
double gyrodet_w[3], gyrodet_B[3];

void rBDOT_Computation(void);

void rSunSensorDataProcessing(void);
void rSS_Main_DB_Init(void);
void rSS_Redundant_DB_Init(void);
void rSS_Main_DB_Copy(void);
void rSS_Redundant_DB_Copy(void);
void rSS_Read_Data(double *SS_Data_Addr,unsigned long int *ADC_Data_Addr);

void rIMU_1_DB_Copy(void);
void rIMU_2_DB_Copy(void);
void rIMU_1_DB_Init(void);
void rIMU_2_DB_Init(void);
void rIMU_Angle_Reset(void);
void rIMUDataProcessing(void);
void rPOR_IMU_Parameters_Init(void);
void rIMU1_DB_Execute();
void rIMU2_DB_Execute();
double rTheta_Limit(double inter_theta);
void rSl_Ecl_OnBrd_detection(void);

#endif // ADCS_SENSORDATAPROCS_H_INCLUDED

