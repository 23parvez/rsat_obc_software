#ifndef ADCS_SENSORDATAPROCS_H_INCLUDED
#define ADCS_SENSORDATAPROCS_H_INCLUDED

//Definitions

#define DelThta_LSB_x IMU_DATA[0]
#define DelThta_MSB_x IMU_DATA[1]
#define DelThta_LSB_y IMU_DATA[2]
#define DelThta_MSB_y IMU_DATA[3]
#define DelThta_LSB_z IMU_DATA[4]
#define DelThta_MSB_z IMU_DATA[5]
#define Mag_x IMU_DATA[6]
#define Mag_y IMU_DATA[7]
#define Mag_z IMU_DATA[8]
#define IMU_Temp IMU_DATA[9]
#define IMU_Diag_STS IMU_DATA[10]
#define IMU_Diag_REG IMU_DATA[11]

extern int DelThta_cat_x; //Delta Theta data for x axis after concatenating LSB and MSB bits
extern int DelThta_cat_y; //Delta Theta data for y axis after concatenating LSB and MSB bits
extern int DelThta_cat_z; //Delta Theta data for z axis after concatenating LSB and MSB bits

typedef struct IMU_Database
{
	double DB_w_BiasCor[3]; //Theta Bias Correction
	double DB_B_BiasCor[3]; //Magnetometer data bias correction
	double DB_wSFC_Neg[3]; //Angular rates Negative Scale Factor Correction
	double DB_wSFC_Pos[3]; //Angular rates Positive Scale Factor Correction
	double DB_BSFC_Neg[3]; //Magnetic field Negative Scale Factor Correction
	double DB_BSFC_Pos[3]; //Magnetic field Positive Scale Factor Correction
	double DB_B_MisCor[3][3]; //Angular rates Misalignment Correction
	double DB_w_MisCor[3][3]; //Magnetometer Misalignment Correction
	double DB_GyroLPF[2]; //Angular rates LPF constants
	double DB_MagLPF[2]; //Magnetometer LPF constants
	unsigned long int DB_imu_checksum;
} root_IMU1_Corr,root_IMU2_Corr;

extern root_IMU1_Corr IMU1_Corr;
extern root_IMU2_Corr IMU2_Corr;

typedef struct IMU_READ_DATA
{
	unsigned long int IMU_DATA[12];
}root_IMU1_DATA,root_IMU2_DATA;

extern root_IMU1_DATA IMU1_DATA;
extern root_IMU2_DATA IMU2_DATA;

typedef struct SunSensor_Database
{
	double DB_Imax_RPD;
	double DB_Imax_RND;
	double DB_Imax_RPND;
	double DB_Imax_RNND;
	double DB_Imax_PP;
	double DB_Imax_PN;
	double DB_Imax_YP;
	double DB_Imax_YN;
	double DB_misaln_cor125[3][3];
	double DB_misaln_cor126[3][3];
	double DB_misaln_cor325[3][3];
	double DB_misaln_cor326[3][3];
	double DB_misaln_cor345[3][3];
	double DB_misaln_cor346[3][3];
	double DB_misaln_cor145[3][3];
	double DB_misaln_cor146[3][3];
	unsigned long int DB_checksum_SS;

} root_SS_Main_2Exe_DB,root_SS_Redundant_2Exe_DB;

extern root_SS_Main_2Exe_DB SS_Main_2Exe_DB;
extern root_SS_Redundant_2Exe_DB SS_Redundant_2Exe_DB;

extern double B_BODYtesla[3], Bsq, B_BODY[3], B_BODYn[3];
extern double magRoll_angle, magPitch_angle, magYaw_angle;
extern double w_BODY[3], w_BODYdeg[3], Thta_rawdata[3], w_BODYnorm;
extern double DelThta_rawdata[3], w_ABC[3], w_BiasCor[3], w_ASC[3], w_AMSC[3],w_RPY[3],w_LPF[3];
extern double B_IMU[3], B_BiasCor[3], B_ASC[3], B_AMSC[3],B_RPY[3],B_LPF[3];
extern double w_BODY_IMU1[3], w_BODY_IMU2[3], B_BODY_IMU1[3], B_BODY_IMU2[3];
extern double IMU_prcd_data[6];
extern double IMU_Sen2Bdy[3][3];
extern double B_BODY_LUT[3];
extern double inter_theta;

extern int PolCheck_LUT, PolChec_LUT_res;

extern int B_rawdata[3]; // Magnetic field raw data
extern int inter_imu_i,inter_imu_j;
extern double Thta_BODY_IMU1[3], Thta_BODY_IMU2[3];

extern unsigned int imu1_db_checksum_obc;
extern unsigned int imu2_db_checksum_obc;


// Sun Sensor


//16 Cells' output (Main and Redundant)
extern double SS_M1, SS_M2, SS_M3, SS_M4, SS_M5, SS_M6, SS_M7, SS_M8;

extern double SB_MAIN[3], SB_RED[3], SS_prcd_data[3];
//6 Active sensors' output after multiplying by constants
extern double ss_temp[3];
extern double SC1, SC2, SC3, SC4, SC5, SC6, SC7, SC8;

//6 Cells' Imax Factors
extern double SC1ImaxF, SC2ImaxF, SC3ImaxF, SC4ImaxF, SC5ImaxF, SC6ImaxF;

extern double comb1, comb2, temp11, temp12;

extern double ele1, az1, ele2; //Elevation and azimuth angles

//6 Active sensors' output after multiplying by constants

extern int TC_SS_Main_Cells_Sel, Panel_Deployment;

extern double sun_sf[3],S_BODY[3], S_BODYn[3]; //SunSensor data in sensor frame

extern double ss2b[3][3];

extern double Ang_Deviation; //arccos(dot(S_BODY,[0,-1,0]))

extern double Roll_ang_err, Yaw_ang_err;
extern int sun_quadrant;
extern int f_Sunlit_Presence, f_Sunlit_Presence_previous;
extern int f_aft_statn_wait,aft_statn_cnt;
extern double SS_Data[16], SSMAIN_ARRAY[8], SSREDT_ARRAY[8];
extern int inter_sunsensor_i,inter_sunsensor_j;

extern int i_MatEq, j_MatEq;

extern int SunNPP_SMtransit_counter;
extern int SunNPP_SMtransit_count_limit;
extern int SunNPP_SMtransit;

extern double AngDev_SAMtransit_thrsld;
extern int SunNPP_SAMtransit_counter;
extern int SunNPP_SAMtransit_count_limit;
extern int SunNPP_SAMtransit;



//BDOT Computation

extern int BDOT_Counter;
extern int Det_BDOT_MC_Count;
extern int TC_Det_Bprev_Count;
extern int TC_BDOT_Compute_Count;
extern double BDOT[3],Bpresent[3];
extern double Bprev[3];
extern double BDOT_deltaT;

extern void rSunSensorDataProcessing(void);
extern void rIMUDataProcessing(void);
/*static void rSS_Read_Data(unsigned long int *ADC_Data_Addr);
static void rSunSensorVectorComp(double SS_ARRAY[8], struct SunSensor_Database *SS_2Exe_Addr);
static void rIMUDataCorrection(struct IMU_READ_DATA* IMU_DATA_ADDRS,struct IMU_Database* IMU_CORR);
static double rTheta_Limit(double var_theta_lim);*/
extern void rBDOT_Computation(void);


#endif // ADCS_SENSORDATAPROCS_H_INCLUDED

