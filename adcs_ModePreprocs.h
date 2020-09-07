#ifndef ADCS_MODEPREPROCS_H_INCLUDED
#define ADCS_MODEPREPROCS_H_INCLUDED

/*static void rSuspended_ModePreprocessing(void);
static void rDetumbling_ModePreprocessing_GYRO_Logic(void);
static void rDetumbling_ModePreprocessing_BDOT_Logic(void);
static void rSunAcquisition_ModePreprocessing(void);
static void rThreeAxis_ModePreprocessing(void);
static void rSafeMode_Preprocessing(void);*/
extern void rScModeSelection(void);


extern int Susp_cnt;

//Detumbling_ModePreprocessing

extern int GYRO_max_threshold_count;
extern int Torquer_Shutdown;

extern int BDOT_Threshold_Count;

//rDetumbling_ModePreprocessing_GYRO_Logic

extern double Tdet[3];
extern int GYRO_Threshold_Count;
extern double TC_GYRO_Det_Min_Thresh;

extern int GYRO_Counter;
extern double gyrodet_w[3];
extern double gyrodet_B[3];



//Rate reduction routine
extern int BDOT_Reduction_Count, BDOT_Norm_Count;
extern double BDOTnorm;
extern double TC_BDOT_Norm_Threshold;
extern double TC_BDOT_Det_Thresh;

//SunAcquisition_ModePreprocessing

extern int SunAcquisition2ThreeAxis_autotransit;
extern double SunAcq_Ang_Thres;
extern int SunAcq3ThreeAx_trsit_cnt;
extern int SunAcq2ThreeAx_trsit_cnt_thres;
extern int SunAcq2DetMode_counter;

extern int ThreeAxis2DetMode_counter, f_threeaxis2safe;

// Timer based sunlit/eclipse

extern int entrytime2eclipse;

// Autotransit

extern int TC_AutoTransitAnyMode2Det;

//for gain select
extern double TC_detumbling_rate_gain[3];
extern double TC_detumbling_bdot_gain[3];
extern double TC_comd_pitch_rate;

extern int i_pini, j_pini;

// Control Bytes

extern unsigned int CB_Detumbling_Mode;
extern unsigned int CB_Q_propagation;
extern unsigned int CB_DAD_quest;
extern unsigned int CB_QuestDataProcessing;
extern unsigned int CB_ErrorComputation;
extern unsigned int CB_ExtendedKalmanFilter;
extern unsigned int CB_LinearController;
extern unsigned int CB_Wheel_OverSpeed_TorqueCutOff;
extern unsigned int CB_DutyCycleGeneration;
extern unsigned int CB_AngularMomentumDumping;
extern unsigned int CB_SpeedBasedMomentumDumping;
extern unsigned int CB_Wheel_Dynamic_Friction;
extern unsigned int CB_Wheel_Spin_updown;
extern unsigned int CB_Wheel_Auto_Reconfiguration;
extern unsigned int CB_Two_RW_control;
extern unsigned int CB_Torquer_Polarity_Check;
extern unsigned int CB_MagFieldComp;
extern unsigned int CB_Sun_model;
extern unsigned int CB_ReferenceQuatComputation;
extern unsigned int CB_RefVectorGeneration;
extern unsigned int CB_RefRate_Computation;
extern unsigned int CB_Sl_Ecl_OnBrd_detection;
extern unsigned int CB_IMUDataProcessing;
extern unsigned int CB_SunSensorDataProcessing;
extern unsigned int CB_BDOT_Computation;

// Mode select
enum SpaceCraftMode_Select
{
	Suspended_ModePreprocessing,
	Detumbling_ModePreprocessing_BDOT,
	Detumbling_ModePreprocessing_GYRO,
	SunAcquisition_ModePreprocessing,
	ThreeAxis_ModePreprocessing,
	Safe_ModePreprocessing
};
extern enum SpaceCraftMode_Select Spacecraft_Mode;

// Power on

void rADCS_Pon_vars(void);

#endif // ADCS_MODEPREPROCS_H_INCLUDED
