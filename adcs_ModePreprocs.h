#ifndef ADCS_MODEPREPROCS_H_INCLUDED
#define ADCS_MODEPREPROCS_H_INCLUDED

void rDetumbling_ModePreprocessing_GYRO_Logic(void);
void rDetumbling_ModePreprocessing_BDOT_Logic(void);
void rThreeAxis_ModePreprocessing(void);
void rAnyMode2DetMode_transit(void);
void rSunAcquisition_ModePreprocessing(void);
void rSafeMode_Preprocessing(void);
void rScModeSelection(void);

double TC_GYRO_Det_Min_Thresh;
int GYRO_Threshold_Count;

//int Detumbling_ModePreprocessing_BDOT;
//int Detumbling_ModePreprocessing_GYRO;

int Susp_cnt;
//int Suspended_ModePreprocessing;
int eSpaceCraftMode, eSuspendedMode, eDetumblingMode, eSunAcquisitionMode, eThreeAxisControlMode, eSafeMode;

///Detumbling_ModePreprocessing

int GYRO_max_threshold_count;
int Torquer_Shutdown;

double BDOT[3];
int BDOT_Threshold_Count;
int TorquerPolaritySetFlag;
//int SunAcquisition_ModePreprocessing;
double TC_Bdot_Gain[3];


///rDetumbling_ModePreprocessing_GYRO_Logic

double Tdet[3];
int GYRO_Threshold_Count;
double gyrodet_w[3];

///////////////BDOT Computation

int BDOT_Counter;
double Bpresent[3];
double Bprev[3];
double BDOT_deltaT;

///GYRO ext Computation

int GYRO_Counter;
double gyrodet_w[3];
double gyrodet_B[3];



///Rate reduction routine
int BDOT_Reduction_Count, BDOTNormCount;
double BDOTnorm;

///SunAcquisition_ModePreprocessing

int SunAcquisition2ThreeAxis_autotransit;
double SunAcq_Ang_Thres;
int SunAcq3ThreeAx_trsit_cnt;
int SunAcq2ThreeAx_trsit_cnt_thres;
int SunAcq2DetMode_counter;

int ThreeAxis2DetMode_counter, f_threeaxis2safe,f_battery_safemode;

/// Timer based sunlit/eclipse

float entrytime2eclipse, orbit_time;

/// Autotransit

int TC_AutoTransitAnyMode2Det;



#endif // ADCS_MODEPREPROCS_H_INCLUDED
