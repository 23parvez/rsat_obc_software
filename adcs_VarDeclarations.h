#ifndef ADCS_VARDECLARATIONS_H_INCLUDED
#define ADCS_VARDECLARATIONS_H_INCLUDED

//HILS_test
#pragma pack(1)
union HILS_test
{
	unsigned char HILS_data_8bit[64];
	unsigned int HILS_data[16];
	unsigned short HILS_data_16bit[32];
	struct
	{
		unsigned short header;
		unsigned char len;
		unsigned char aux;
		unsigned char mode_flag;
		unsigned short mag_field[3];
		unsigned char polarity;
		unsigned int rw_torque[4];
		unsigned int Mic_time;
		unsigned char fillerbyte[27];
		unsigned char reserved_byte;
		unsigned short checksum;
		unsigned short Footer;
	};
}HILS_packet;
extern unsigned short hils_mode_select;
extern void rHILS_payload(union HILS_test* HILS_packets);
extern void rHILS_packets();

#define abs_f(a) (((a) < 0.0) ? (-1.0 * (a)) : (a))
#define abs_i(a) (((a) < 0) ? (-1 * (a)) : (a))
#define sign_f(a) (((a) < 0.0)? (-1.0) : (1.0))
#define sign_i(a) (((a) < 0)? (-1) : (1))

#define max_fun3(a,b,c) ((a) > (b)) ? (((a) > (c)) ? (a) : (c)) : (((b) > (c)) ? (b) : (c))
#define min_fun3(a,b,c) ((a) < (b)) ? (((a) < (c)) ? (a) : (c)) : (((b) < (c)) ? (b) : (c))
#define min_fun2(a,b) ((a) < (b)) ? (a) : (b)

//#define CB_Q_propagation 1

#define True 1
#define False 0
#define Enable 1
#define Disable 0
#define DIVIDE_BY_ZERO 0.0000000000000001
#define Set 1
#define Reset 0

#define IMU1 1
#define IMU2 0

#ifndef NULL
#define NULL ((void*)0)
#endif

#define Cur_Positive 1
#define Cur_Negative -1
#define No_Current 0

#define TC_Main_Cells 0
#define TC_Redundant_Cells 1

#define TC_Not_Deployed  3
#define TC_PosR_Deployed 2
#define TC_NegR_Deployed 1
#define TC_All_Deployed  0

#define IMU1 1
#define IMU2 0

#define RWHEEL0 0
#define RWHEEL1 1
#define RWHEEL2 2
#define RWHEEL3 3

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
enum SpaceCraftMode_Select Spacecraft_Mode;

extern int i_pini, j_pini;

///Orbit Model
extern const double c_twomu;
extern const double c_tsince_min;
extern const double c_OMEGAE;
extern const double c_vkmpersec;
extern const double c_radiusearthkm;
extern const double c_auer;
extern const double c_au;
extern const double c_j2;
extern const double c_Twopi;
extern const double c_x2o3;
extern const double c_xke;
extern const double c_dividebyzerovalue;
extern const double c_mu;
extern const double c_invmu;
extern const double c_Day_To_Seconds;
extern const double c_Pi;
extern const double c_xpdotp;
extern const double c_ss;
extern const double c_qzms2t;
extern const double c_j3oj2;
extern const double c_j4;
extern const double c_temp4;
extern const double c_min_per_day;
extern const double c_D2R;
extern const double c_R2D;
extern const double c_AS2R;
extern const double c_Mmax;
extern const double c_oneminute;
extern const double c_onesecond;
extern const double c_halfhour;

extern const double fc[5][5];

extern const double nut[106][10];


///IGRF
extern const int c_Nmax;
extern const int c_Kmax;
extern const double c_Pibytwo;
extern const double c_a;
extern const double c_sdmax;
extern const double c_MaC;
extern const double c_MiC;


extern const double c_gval[104];
extern const double c_gsval[104];
extern const double c_hval[104];
extern const double c_hsval[104];

///Sun Model
extern const double c_L_Msun1;
extern const double c_L_Msun2;
extern const double c_Msun1;
extern const double c_Msun2;
extern const double c_L_Ecliptic1;
extern const double c_L_Ecliptic2;
extern const double c_Sun_Dis1;
extern const double c_Sun_Dis2;
extern const double c_Sun_Dis3;
extern const double c_Epsilon1;
extern const double c_Epsilon2;
extern const double c_KDset1;
extern const double c_KDset2;
extern const double c_KDset3;

extern const int c_DPM_Pol_LookUpTable[27][3];

///Kalman Filter
extern const double c_I_nine_cross_nine[9][9];
extern const double c_I_three_cross_three[3][3];
extern const double  c_rk_T[9][9];
extern const double c_rk[9][9];

///16 Imax extern constants of for all sensors (Main and Redundant cells) ( NOTE: extern constant = Overall_Imax / Individual_Imax)
extern const double c_Imax_RPD_Red;
extern const double c_Imax_RND_Red;
extern const double c_Imax_RPND_Red;
extern const double c_Imax_RNND_Red;
extern const double c_Imax_PP_Red;
extern const double c_Imax_PN_Red;
extern const double c_Imax_YP_Red;
extern const double c_Imax_YN_Red;

extern const double c_Imax_RPD_Main;
extern const double c_Imax_RND_Main;
extern const double c_Imax_RPND_Main;
extern const double c_Imax_RNND_Main;
extern const double c_Imax_PP_Main;
extern const double c_Imax_PN_Main;
extern const double c_Imax_YP_Main;
extern const double c_Imax_YN_Main;

///---------------------------------------------------------------------------------------------------------------------------
extern const double c_SSThrsld; ///Sunsensor threshold value
extern const double c_Sunlit_Thrsld; ///Threshold for sunlit detection
extern const double c_AngDev_SMtransit_thrsld; /// (30 degrees) Threshold for sun presence in negative pitch side
extern const double c_AngDev_SAMtransit_thrsld; /// (30 degrees) Threshold for sun presence in negative pitch side

///SunSensor's Misalignment Correction Matrices
extern const double c_misaln_cor125[3][3];
extern const double c_misaln_cor126[3][3];
extern const double c_misaln_cor325[3][3];
extern const double c_misaln_cor326[3][3];
extern const double c_misaln_cor345[3][3];
extern const double c_misaln_cor346[3][3];
extern const double c_misaln_cor145[3][3];
extern const double c_misaln_cor146[3][3];

///Sun Sensor to Body frame conversion for all the 8 quadrants
extern const double c_ss2b1256[3][3];
extern const double c_ss2b2356[3][3];
extern const double c_ss2b3456[3][3];
extern const double c_ss2b4156[3][3];

///Quest
extern const double c_wks_mag;
extern const double c_wkm_mag;
extern const double c_wks_sunmag;
extern const double c_wkm_sunmag;

/// Linear Controller
extern double TC_KR[3];
extern double TC_KP[3];
extern const double c_DPMMAX;
extern const double c_RPM2RADpS;
extern const double c_RADps2RPM;
extern const double TC_T_RW_MAX;

extern const double c_wh2B_mat_4RW[3][4];

extern const double c_wh2B_mat_1230[3][4];

extern const double c_wh2B_mat_1204[3][4];

extern const double c_wh2B_mat_1034[3][4];

extern const double c_wh2B_mat_0234[3][4];

extern const double c_B2wh_mat_4RW[4][3];


extern const double c_B2wh_mat_1230[4][3];

extern const double c_B2wh_mat_1204[4][3];

extern const double c_B2wh_mat_1034[4][3];


extern const double c_B2wh_mat_0234[4][3];


///ADandEstimation
extern const int c_I_four_cross_four[4][4];
extern const unsigned int c_FFFF;
extern const double c_Resol_DelThta;
extern const double c_Resol_B;
extern const double c_TM_Resol_DelThta;
extern const double c_TM_Resol_w;
extern const double c_TM_Resol_B;
extern const double c_TM_RW_Resol;


///IMU

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

struct IMU_Database
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
} IMU1_Corr,IMU2_Corr;

struct IMU_READ_DATA
{
	unsigned long int IMU_DATA[12];
}IMU1_DATA,IMU2_DATA;

extern double TC_wAD_BODYmaxThRoll, TC_wAD_BODYmaxThPitch, TC_wAD_BODYmaxThYaw, TC_wAD_BODYminThRoll, TC_wAD_BODYminThPitch, TC_wAD_BODYminThYaw;
extern int wAD_updatecount, TC_wAD_updateTimeThresh;
extern double B_BODYtesla[3], Bsq, B_BODY[3], B_BODYn[3];
extern double magRoll_angle, magPitch_angle, magYaw_angle;
extern double w_BODY[3], w_BODYdeg[3], Thta_rawdata[3], w_BODYnorm;
extern double DelThta_rawdata[3], w_ABC[3], w_BiasCor[3], w_ASC[3], w_AMSC[3],w_RPY[3],w_LPF[3];
extern double B_IMU[3], B_BiasCor[3], B_ASC[3], B_AMSC[3],B_RPY[3],B_LPF[3];
extern double w_BODY_IMU1[3], w_BODY_IMU2[3], B_BODY_IMU1[3], B_BODY_IMU2[3];
extern double IMU_prcd_data[6];
extern double* IMU_prcd_data_ptr;
extern double IMU_Sen2Bdy[3][3];
extern double B_BODY_LUT[3];

extern int PolCheck_LUT, PolChec_LUT_res;
extern double MagBias_residue_LUT[27][3], MagBias_act_LUT[27][3];

extern int B_rawdata[3]; // Magnetic field raw data
extern int inter_imu_i,inter_imu_j;
extern double Thta_BODY_IMU1[3], Thta_BODY_IMU2[3];

extern unsigned int imu1_db_checksum_obc;
extern unsigned int imu2_db_checksum_obc;

/// IGRF model

extern int i_rfc, j_rfc;

extern double temp_magad;
extern double BNorth,BEast,BNorth_old,BDown,rot_ang,A1_magad,A2_magad,A3_magad,B_NED[3],B_ECI[3],B_ECIn[3],B_ECItesla[3];
extern double co_dec;
extern int fN_MAGAD,gN_MAGAD,gmm;
extern double st_magad,ct_magad,sl_magad[14],cl_magad[14];
extern double one_magad,two_magad,sd_magad,p_magad[106],q_magad[106];
extern int I_MFC,K_MAGAD,I_MAGAD,J_MAGAD,M_MAGAD,N_MAGAD;
extern double Cond1;
extern double B_ECEF[3];
extern double A_R, Alti_Mod, Rho, B_Rho, cd_magad, old_cos, s_magad;
extern unsigned char m_gh, n_gh, i_gh, j_gh;
extern double g[15][16], h[15][16];
extern double REF_FRAME_DCM[3][3];
extern int DeltaT_MFC, DeltaT_Updated;

///Sun Model
extern double S_ECI[3], S_ECIn[3];
extern double L_Msun, Msun, L_Ecliptic, Sun_Dis, Epsilon;

/// Ref quaternions
extern double SUN_ECI_mag, X_SVO2ECI_mag, Z_SVO2ECI_mag, Y_EPO2ECI_mag, X_EPO2ECI_mag, Z_SFAO2ECI_mag, X_SFAO2ECI_mag;
extern double X_SFDO2ECI_mag, Z_SFDO2ECI_mag;
extern double Y_SVO2ECI[3], Z_SVO2ECI[3], X_SVO2ECI[3], R_SVO2ECI[3][3], Q_SVO2ECI[4];
double X_MDO2ECI[3],Y_MDO2ECI[3],Z_MDO2ECI[3],R_MDO2ECI[3][3],Q_MDO2ECI[4];
extern double Z_EPO2ECI[3], Y_EPO2ECI[3], X_EPO2ECI[3], R_EPO2ECI[3][3], Q_EPO2ECI[4];
extern double Y_SFAO2ECI[3], Z_SFAO2ECI[3], X_SFAO2ECI[3], R_SFAO2ECI[3][3], Q_SFAO2ECI[4];
extern double Y_SFDO2ECI[3], Z_SFDO2ECI[3], X_SFDO2ECI[3], R_SFDO2ECI[3][3], Q_SFDO2ECI[4];

extern double R_MDO_CB[3][3];

extern double rdotrst, SAT_ANGLE_STAT;
extern double STATION_ECEF[3], STATION_ECI[3], STATION_vector[3], STATION_ECIn[3];
extern double X_SPO2ECI[3], Y_SPO2ECI[3], Z_SPO2ECI[3];

extern double TC_eesqrd;
extern double TC_long_station, TC_lat_station;

extern double X_SPO2ECI_mag, Y_SPO2ECI_mag;
extern double R_SPO2ECI[3][3], Q_SPO2ECI[4];

///Ref Vector Generation
extern double Q_REF[4], Q_REF_conj[4], Q_StP2ECI[4], R_StP2ECI[3][3];
extern double Q_svn_off[4], Q_stn_off[4], Q_REF_GND[4];
extern double B_REF[3], S_REF[3], B_REFn[3], S_REFn[3];

/// Ref Gyro

extern double w_REF[3], w_REF_prev[3];
extern double Q_REF_pres[4], Q_REF_prev[4], Q_REF_prev_conj[4];
extern double Q_REF_diff[4], QRD_vect_norm;
extern double Q_angle, Q_axis[3];

/// Onboard Eclipse Algorithm
extern double theta1_se, theta2_se;
extern double psi_sl_ecl, elapsed_running_timer;
extern int f_Sunlit_Presence_orbit, f_Sunlit_Presence_timer, f_Sunlit_Presence_sensor;

extern int f_station_tracking_enabled,f_station_tracking_enabled_pre;
extern double rsun[3], rsat[3], magrsun;

extern double tempse;
extern double bsqrd;
extern double asqrd;
extern double adotb;
extern double distsqrd;
extern double tmin;

///Ref Vector Generation
extern double Q_REF[4], Q_StP2ECI[4], R_StP2ECI[3][3];
extern double B_REF[3], S_REF[3], B_REFn[3], S_REFn[3];

///To be deleted
extern int TC_EarthPointingFrame;
extern int TC_SunEarthVaryingFrame;
extern int TC_SunFixedAscendingFrame;
extern int TC_SunFixedDescendingFrame;
extern int TC_StationTracking;

///----------AD and Estimation---------------

/// Quest Data Processing
extern int sc_qst; ///sample count for magAD data processing(MC)
extern int dc_qst; ///data count for magAD data processing (sec) NOTE: NOT MC
extern int wc_qst; ///wait period count for magAD data processing (sec) NOTE: NOT MC
extern int mat_mag_DataCounter, mat_sm_DataCounter; ///data counter for magAD and sunmagAD to track the filling matrices
extern int TC_wp_QDP; ///Telecommand for Wait period selection in QDP
extern double NMB_mag[3][24]; ///Measurement Matrix (Magnetic field) for magAD
extern double NRB_mag[3][24]; ///Reference Matrix (Magnetic field) for magAD
extern double NMB_sunmag[3][8]; ///Measurement Matrix (Magnetic field) for sunmagAD
extern double NRB_sunmag[3][8]; ///Reference Matrix (Magnetic field) for sunmagAD
extern double NMS[3][8]; ///Measurement Matrix (sun sensor) for sunmagAD
extern double NRS[3][8];  ///Reference Matrix (sun model) for sunmagAD
extern int f_DataSort_MAG,f_DataSort_SUNMAG; ///Flags that are raised to tell data matrices are ready for Quaternion computation in QUEST
extern int i_QDP;
extern int CB_Q_propagation, OBC_Quest_update, Quest_update_available, w_q_update_satisfy;

extern int CB_DAD_quest;
extern double B_DAD[3][3], Bt_DAD[3][3];
extern double NRBt_mag[24][3], NRSt[8][3];
extern double NRBt_sunmag[8][3];
extern double wkm, wks;
extern double S_DAD[3][3], St_DAD[3][3], S_sqr_DAD[3][3];
extern double Z_DAD[3], Zt_DAD[3];
extern double sigma_DAD, delta_DAD, a_DAD, b_DAD,c_DAD,d_DAD, k_DAD;

extern double root_DAD, func_DAD, Dfunc_DAD, func_prev_DAD;
extern double alpha_DAD, beta_DAD, gamma_DAD;
extern double X_DAD[3];

extern double Q_quest_DAD[4];
extern double TC_magMin_angle, TC_magMax_angle;
extern double Qquest_update[4];

extern int I_DAD, J_DAD, K_DAD;
extern double NMB_NRBt[3][3], NMS_NRSt[3][3];
extern double Zt_Z_DAD, Zt_S_Z_DAD, Zt_Ssqr_Z_DAD;
extern double alpha_I_DAD[3][3], beta_S_DAD[3][3], sumX_DAD[3][3];
extern int TC_SunMagAD;
extern double trace_S_DAD, adj_S_DAD[3][3];

/// Qgyro

extern double Del_Y_theta, Del_R_theta, Del_P_theta; ///Small angle obtained by extern integrating Gyro data over a periof of time (mc/MC)
extern double Del_Q[4]; ///Delta Qs
extern double q_prop_out[4],Qprop_prev[4];
extern double Qbody[4];

extern double W_cross[3][3];
extern double big_omega[4][4];
extern double Delta_by2;
extern double W_norm;
extern double Delta;
extern double cos_of_W_norm_and_Delta_by2;
extern double cos_of_W_norm_Delta_by2_with_I[4][4];
extern double sin_of_W_norm_and_Delta_by2;
extern double sin_of_W_norm_Delta_by2_and_W_norm;
extern double sin_of_W_norm_Delta_by2_W_norm_and_big_omega[4][4];
extern double cos_and_sin[4][4];


///New variables

extern int GPSDataReady_NA_count;
extern int TC_GPS2TLE_Switch;
extern int Present_OBT;
extern int OBT_at_TLE_epoch;
extern int Delta_TLE;
extern int TC_GPSvalidity_Threshold;

//GPS
extern unsigned char* GPS_TM_Buffer_Addr_USC;
extern unsigned long int GPS_Data_Read_Status;
extern unsigned long int GPS_Buffer_Data[106];
extern unsigned long int GPS_RCVD_DATA[106];
extern unsigned char GPS_obc_checkum;
extern unsigned int f_GPS_Valid_Data;



///Julian Day
extern double jd_time, tut;
extern double pps_deltaT, Julian_day;

///ast_args
extern double tt[4], f[5];

///main
extern int i_god, j_god;
extern int minorcyclecount, majorcyclecount;
extern int k, i, j, ktr, TLE_Select;

///rOrbitalElements_generation_GPS
extern int GPS_Select, GPSDataReady, TC_GPS_pulse_duration;
extern int i_jday, lmonth[13], year_GPS, UTC_mon_GPS, tempdays, Numofdays, UTC_day_GPS, UTC_hr_GPS, UTC_min_GPS;

extern double epochdays_GPS, UTC_sec_GPS, PosX_ECEF_GPS, PosY_ECEF_GPS, PosZ_ECEF_GPS, Pos_ECEF_GPS[3];
extern double VelX_ECEF_GPS, VelY_ECEF_GPS, VelZ_ECEF_GPS, Vel_ECEF_GPS[3], Vel_ECI_temp[3], Vel_ECI_GPS[3], Pos_ECI_GPS[3];
extern double rpef[3], vpef[3], wcrecef[3], e_vec[3], angmomentumvec[3];
extern int Epochyear_GPS;


extern double radialdistance, radialdistance_ecef, velmag, r_delta, velmagsq, rdtv, ecc_temp1, ecc_temp2;
extern double ecc_temp3, ecc_temp4, ecc_temp5, ecc_temp6, ecc_temp7, ecc_GPS, semi_den, semimajoraxis_GPS, Alti_GPS;
extern double angmomentumvecmag, delta_hmag, invangmomentumvecmag;
extern double inclination_GPS, sinlongacnode, coslongacnode, nodeo_GPS, e_vecdtr, tempta, trueanomoly_GPS, Ndte_vec, N_ecc;
extern double temparg, argpo_GPS, omecc, sine, cose, eccanomaly_GPS, mo_GPS, Orbit_Period, no_GPS, longitude_GPS, latitude_GPS;

///Orbital Elements generation TLE
extern double sec_TLE_tc, no_TLE_tc, ecc_TLE_tc, trueanomoly_TLE_tc, epochdays_TLE_tc;
extern int minute_TLE_tc, hr_TLE_tc, day_TLE_tc, mon_TLE_tc, year_TLE_tc, Epochyear_TLE_tc, TC_TLE_Elements_select;
extern int epochyr, dayofyr, inttemp;
extern double epochdays, temp_jday, ibexp_TLE_tc, bstar_TLE_tc, inclination_TLE_tc, nodeo_TLE_tc, argpo_TLE_tc;
extern double mo_TLE_tc, ibexp_tc, bstar_tc, inclo_tc, nodeo_tc, argpo_tc, mo_tc;

///Orbit Initialization and Propagation
extern double Tsince_GPS, epochdays_sel, inclination_sel, nodeo_sel, trueanomoly_sel, mo_sel, argpo_sel;
extern double ecc_sel, no_sel, ibexp_sel, bstar_sel, sec_sel;
extern int year_sel, mon_sel, days_sel, hr_sel, minute_sel, epochyr_sel;
extern int GPS_Elements_Available, TLE_Data_Available;
extern double rtom, ao_sfour, pow_psisq, psetasq, intermediate, am_den, temp_temp, temp1, wcreci[3];
extern double Tsince, Tsince_TLE;
extern double Tsince_TLE_tc, Day_Of_Year_DeltaT, Delta_T, Orbit_Period_Comp, wo, inclo, nodeo, argpo, mo, ecco, no, ibexp, bstar;
extern int CB_OrbitModel, OrbitModel_Start;
extern char chksum_tle;

extern double con41, cc1, cc4, cc5, d2, d3, d4, delmo, eta, argpdot, omgcof, sinmao, aycof, t2cof, t3cof;

extern double t4cof, t5cof, x7thm1, mdot, nodedot, xmcof, nodecf, cc3, var;

extern double uuu, temp_prop, var1;


extern double eccsq, omeosq, rteosq, cosio, cosio2;

extern double ak, d1,  del, adel, ao, sinio, po, con42, posq, rp;

extern double sfour, qzms24;

extern double pinvsq, tsi, etasq, eeta, psisq,  coef, cc2, coef1,qzms24_temp;

extern double cosio4, temp1_or_init, temp2, temp3, temp_or_init, temp_om;

extern double xhdot1, xlcof;

extern double cc1sq, ak_temp, ao_temp, delmo_temp;

extern double x1mth2, perigee;

extern double xmdf, argpdf, nodedf, mm, t2, tempa, tempe, delomg, delm, t3, t4, nm, em, inclm, ecose, am, argpm, nodem, xlm, sinim, cosim;
extern double ep, xl, sinip, cosip, xincp, argpp, nodep, mp, axnl, aynl, esine, r1;
extern double betal, sinu, cosu, suu, sin2u, cos2u, mrt, xnode, xinc, mvt, rvdot, emsq, sinsu, cossu, snod, cnod, sini, cosi;
extern double xmx, xmy, ux, uy, uz, vx, vy, vz,  xsatx_eci, xsaty_eci, xsatz_eci, vsatx_eci, vsaty_eci, vsatz_eci;
extern double templ, am_temp, nm_temp, mm_temp, u_temp, el2, pl, rl, rdotl, rvdotl;
extern double sinkp, coskp, eol, etempe;
extern double del_temp;
extern double Pos_ECI[3], Vel_ECI[3], Pos_ECEF[3], Vel_ECEF_temp[3], Vel_ECEF[3];
extern double Pos_ECIn[3], Vel_ECIn[3], Pos_ECEFn[3], Vel_ECEFn[3];

///Orbital elements computation
extern double semimajoraxis, ecc, Alti, inclination_temp, inclination, RAAN, trueanomoly, argofperigee, eccanomaly, ecc_r;
extern double longitude, latitude_temp, latitude ,longitude_tan_num, longitude_tan_den;
extern double xa_gcgd, mua_gcgd,ra_gcgd,l_gcgd,dlambda_gcgd,h_gcgd,den_gcgd,rhoa_gcgd,dmu_gcgd,gd_gcgd;

///Ecef to ECI to ecef
extern double UT1, UTC, TC_delUT1, TAI, JDTDT, M_quad, sine1, sine2, TDB, TDT, TTDB, JDTDB, TTDB2, TTDB3, zeta, z, theta;
extern double pre_temp[3][3], precession[3][3], eps, dpsi, nut_temp[3][3], xin_temp, nutation[3][3], tut1, gmst0, gmst_, gast;
extern double sidereal[3][3], TC_xp, TC_yp, polarmotion[3][3], nut_sid_temp[3][3], ECEFtoECI[3][3], ECItoECEF[3][3],ang;
extern double TC_delAT, TTDT, deps;

///NED to ECEF
extern double NEDtoECEF[3][3];

/// Sun Sensor

struct SunSensor_Database
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

} SS_Main_DB,SS_Redundant_DB,SS_Main_2Exe_DB,SS_Redundant_2Exe_DB;

///16 Cells' output (Main and Redundant)
extern double SS_M1, SS_M2, SS_M3, SS_M4, SS_M5, SS_M6, SS_M7, SS_M8;

extern double *SS_prcd_data_ptr;
extern double SB_MAIN[3], SB_RED[3], SS_prcd_data[3];
///6 Active sensors' output after multiplying by constants
extern double ss_temp[3];
extern double SC1, SC2, SC3, SC4, SC5, SC6, SC7, SC8;

///6 Cells' Imax Factors
extern double SC1ImaxF, SC2ImaxF, SC3ImaxF, SC4ImaxF, SC5ImaxF, SC6ImaxF;

extern double comb1, comb2, temp11, temp12;

extern double ele1, az1, ele2; ///Elevation and azimuth angles

///6 Active sensors' output after multiplying by constants

extern int TC_SS_Main_Cells_Sel, Panel_Deployment;

extern double sun_sf[3],S_BODY[3], S_BODYn[3], B_BODYn[3]; ///SunSensor data in sensor frame

extern double ss2b[3][3];

extern double Ang_Deviation; ///arccos(dot(S_BODY,[0,-1,0]))

extern double Roll_ang_err, Yaw_ang_err;
extern int sun_quadrant;
extern int f_Sunlit_Presence, f_Sunlit_Presence_previous;
extern int f_aft_statn_wait,aft_statn_cnt;
extern double SS_Data[16];
extern int inter_sunsensor_i,inter_sunsensor_j;

extern int i_MatEq, j_MatEq;
extern int Sunlit_presence_timer, Eclipse_presence_timer;

extern int SunNPP_SMtransit_counter;
extern int SunNPP_SMtransit_count_limit;
extern int SunNPP_SMtransit;

extern double AngDev_SAMtransit_thrsld;
extern int SunNPP_SAMtransit_counter;
extern int SunNPP_SAMtransit_count_limit;
extern int SunNPP_SAMtransit;

//Checksum
unsigned long int ss_main_db_checksum_obc;
unsigned long int ss_redundant_db_checksum_obc;

extern int i_comr, j_comr, k_comr;
///Rotation matrices
extern double Rx[3][3], Ry[3][3], Rz[3][3];

///mat inverse
extern double determinant, Invmatout33[3][3];

///mat multiplication
extern double Matout31[3], Matout33[3][3], Matout441[4];

///Cross product
extern double Cross_Product[3];

///Vector Normalization
extern double Norm_out[3], vecnorm_mag;
extern double out_Quat_norm[4]; ///Global variable (Normalized Quaternion)
extern double mat_mult[3][3]; ///Global variable (matrix multipication of 2 matrices[3][3])
extern double mat_adj[3][3]; ///Global variable (Adjoint of matrix[3][3])
extern double DCM2Q_out[4]; ///Global variable (RM to Quaternion Conversion)
extern double out_quatMult[4]; ///Global variable (Multiplication of 2 Quaternions)
extern double Matout341[3];
extern double Matout431[4];
extern double out_Quat_mult[4]; ///Global Variable of the Out_Quat from Q Multiplication Routine

///BDOT Computation

extern int BDOT_Counter;
extern int Det_BDOT_MC_Count;
extern int TC_Det_Bprev_Count;
extern int TC_BDOT_Compute_Count;
extern double BDOT[3],Bpresent[3];
extern double Bprev[3];

///Det gyro

extern int TC_Det_GYRO_Compute_Count, GYRO_Counter;
extern double gyrodet_w[3], gyrodet_B[3];


///Dutycycle Generation
extern int DutyCycleGenEnable, TorquerPolaritySetFlag;
extern double MR, MP, MY;
extern int Roll_MTR_Pol_Reversal, Pitch_MTR_Pol_Reversal, Yaw_MTR_Pol_Reversal;
extern int DPM_Polarity[3], Ton[3], Toff[3], DPM_Pol_prev[3];
extern int Roll_MTREnable, Pitch_MTREnable, Yaw_MTREnable;
extern int MTR_ActuationCycle;

extern double TorquerDutyCycle[3];
///extern double DutyCycle_Roll;
///extern double DutyCycle_Pitch;
///extern double DutyCycle_Yaw;
extern float ActuationCycle;

///Linear controller
extern int i_lict, j_lict;
extern float RW_Wheel_Speed[4];
extern double TC_KR[3];
extern double TC_KP[3];
extern double TC_wh_speed_cutoff;
extern double Tc[3],Qerror[4];
extern double RWSpeed[4], v0_rad_sec[4];
extern double H_wh[4], HB[3];
extern double T_RW[4];
extern int CB_Wheel_OverSpeed_TorqueCutOff, DFriction_Compensation;
extern double B2wh_mat[4][3], wh2B_mat[3][4];
extern int k;
extern int TachoHyst[4], DFCCount[4], FirstSpeed[4];
extern double DFCGainHigh, LPFK1HighSL, LPFK2HighSL, DFCGainLow, LPFK1LowSL, LPFK2LowSL;
extern double ExWhMom[4], DFCGainSL[4], LPFK1SL[4], LPFK2SL[4],
DFCGainSL[4], LPFK1SL[4], LPFK2SL[4], WhMom0[4], ActWhMom[4], LossWhMom[4], LossWhMomFilt[4], DFCTorq[4];
extern int DFCCountLimSL[4], DFCCountLimSL[4], DFCcountHigh, DFCcountLow;
extern int wheel_spin_logic, spin_up_avg_count, spin_up_avg_count_2;
extern double del_v0[4], del_v0a, T_RW_spin[4];
extern int wheel_index[4], wheel_index_ARCsum, Wheel_Config, RW_ARC_Logic, RW_ARC_Count, TC_RW_ARC_Count_thres, count_arc_w0, TC_ARC_Time_Cycle, count_arc_w1, count_arc_w2, count_arc_w3;
extern double pres_exp_whsp_ch[4], exp_whsp_ch[4], ch_obs_whsp[4], prev_obs_whsp_ch[4], diff_obs_exp_ch[4];
extern double TC_ARC_RPM_Thres;
extern double speedDFCch;

extern float TC_RW_Nominal[4];

extern const double c_MOI_wh1, c_MOI_wh2, c_MOI_wh3, c_MOI_wh4, c_MOI_wh;

///Angular Momentum Dumping
extern int f_Momentum_Dumping;
extern int dumping_on;
extern double delta_HB[3];
extern double TC_Hmin;
extern double TC_Hmax;
extern double DPM[3];
extern double TC_MDk;

///Speed based Momentum Dumping
extern double TC_max_whsp_spdump, TC_min_whsp_spdump, TC_SpeedDumpLimit, MOI_wh;
extern double TC_SpeedDumpTime;
extern int check_dump_wh[4];
extern double RWSpeed[4], H_retn;
extern double T_RW_sdump[4];
extern int TC_SpeedAngularMomemtumDumping, wh_sdump_start[4], speed_based_torquer_control;
extern double TB_sMD[3], u_parl[3], B_BODYn[3], Tc[3], T_RW[4], T_RW_sdump[4], T_RWB[3];
extern double del_Vw[4];
extern double T_RWBn[3], tau_ms, min_TW, Bsq;
extern double u_perp[3];
extern double Pse_Inv_Dist_Mat[4][3];

///Two RW control

extern double Twof_veh[3][3], Tv_wof[3][3], Tcv_temp[3], Tcv[3], Tdeficit_temp1[3], Tdeficit_temp2[3], Tdeficit[3], Tdefnorm;
extern double alpha_2RW_temp[3], alpha_2RW_tempnorm, alpha_2RW, B_Tef_ANG, mu_m_temp[3], mu_m[3];
extern double Tdes[3], T_MT_2RW[3], Trwa_mt_comp[3], Trwa[4], T_SD_RW[4], Tmm_actual[3];


///Kalman Filter
extern int i_kf, j_kf, k_kf;
extern double sigma_v;
extern double sigma_u;
extern double sigma_m;
extern double Qk_temp1;
extern double Qk_temp2;
extern double Qk_temp3;
extern double Qk_temp4;
extern double qk_DCM[3][3];
extern double ref_cross[3][3];
extern double Hk[3][9];
extern double pk_HkT[9][3];
extern double pk_plus_one[9][9];
extern double phi_k_pk_plus[9][9];
extern double phi_k_pk_plus_phi_k_T[9][9];
extern double rk_Qk[9][9];
extern double rk_Qk_rk_T[9][9];
extern double Qk[9][9];
extern double  phi_k_T[9][9];
extern double phi_k[9][9];
extern double F_k1[3][3], mod_of_w_k, jw_cross[3][3], F_k4_temp1[3][3], F_k4_temp2[3][3], F_k4[3][3], F_k[9][9];
extern double phi_12[3][3];
extern double phi_12_temp2[3][3];
extern double Result_of_sine_Delta_cube;
extern double Result_of_sine_Delta;
extern double phi_12_temp1[3][3];
extern double phi_11[3][3];
extern double phi_11_temp4[3][3];
extern double phi_11_temp3[3][3];
extern double cos_of_Wk1_Delta_by_Wk1_square;
extern double Q_quest_DAD_den;
extern double mod_of_Wk1_square;
extern double mod_of_Wk1_cube;
extern double cos_of_Wk1_Delta;
extern double phi_11_temp2[3][3];
extern double phi_11_temp1[3][3];
extern double Wk1_one[3][3];
extern double sine_of_Wk1_Delta_by_Wk1;
extern double sine_of_Wk1_Delta;
extern double mod_of_Wk1;
extern double pk_plus[9][9];
extern double Kk_Hk_with_minus_one[9][9];
extern double Kk_Hk[9][9];
extern double qk_plus[4], qk_plus_conj[4];
extern double qk_plus_temp[4];
extern double Yk[3];
extern double Xk[9];
extern double q_by_two[4][3];
extern double Xk_plus_temp[3];
extern double Xk_plus[9];
extern double Yk_minus_hk_Kk[9];
extern double Yk_minus_hk[3];
extern double Kk_matrix[9][9];
extern double K_temp[3][3];
extern double pk_HkT_Hk_Rk[3][3];
extern double pk_HkT_Hk[3][3];
extern double pk[9][9];
extern double Rk[3][3];
extern double Hk_temp[9][3];
extern double hk[3];
extern double Wk1[3][3];
extern double phi_12_temp3[3][3];
extern double qk_minus[4], hk_xk_minus[3];
extern int TC_KalmanFilter_ENABLE;
extern double sun_noise;
extern double mag_noise;

// Extended Kalman Filter 2

extern int ecl_ekf, kfmeas, tor_counterk;

extern double T_RW_NETk[3], T_MTk[3], Mk[3], T_MTk[3], T_NETk[3], w_k[3], RWSpeedk[4];
extern double b_k[3], lpfx_magbk, lpfy_magbk, lpfz_magbk, lpfb_k[3], lpfb_k_prev[3];
extern double pbk, qqbk;

extern double kfk1[4], kfq_dot[4], kfg1[3], kfw_dot[3], kfv1[4], kfv_dot[4], kfq01[4], kfw01[3], kfv01[4];
extern double kfk2[4], kfg2[3], kfv2[4], kfq02[4], kfw02[3], kfv02[4];
extern double kfk3[4], kfg3[3], kfv3[4], kfq03[4], kfw03[3], kfv03[4];
extern double kfk4[4], kfg4[3], kfv4[4];

extern double kfom[4][4], kfq_dot[4], kfv_dot[4], kfT_RW_NET_temp[4], kfT_RW_NET[3], kfH_wh_ind[4], kfH_wh2body[3], kfI_MAT_w0[3], kfI_MAT_w0_HW2B[3], kfT_NET[3], kfw0crossI_MAT_w0_HW2B[3], kfw_dot_temp[3], kfw_dot[3];
extern int f_EKF2_prop_en,f_EKF2_mag_bias_dis;
extern double I_MAT[3][3], I_MAT_Inv[3][3];
extern double MOI_wh_mat_Inv[4][4];
extern double MOI_wh_mat[4][4];

//extern int ThreeAxis_ModePreprocessing;

///Panel deployment
extern int Not_Deployed;
extern int All_Deployed;
extern int PosR_Deployed;
extern int NegR_Deployed;

///Detumbling

extern double TC_GYRO_Det_Min_Thresh;
extern int GYRO_Threshold_Count;

//extern int Detumbling_ModePreprocessing_BDOT;
//extern int Detumbling_ModePreprocessing_GYRO;

extern int Susp_cnt;
//extern int Suspended_ModePreprocessing;
extern int eSpaceCraftMode;

///Detumbling_ModePreprocessing

extern int GYRO_max_threshold_count;
extern int Torquer_Shutdown;

extern int BDOT_Threshold_Count;
extern int TorquerPolaritySetFlag;
//extern int SunAcquisition_ModePreprocessing;
extern double TC_Bdot_Gain[3];


///rDetumbling_ModePreprocessing_GYRO_Logic

extern double Tdet[3];
extern int GYRO_Threshold_Count;

///////////////BDOT Computation

extern int BDOT_Counter;
extern double Bpresent[3];
extern double BDOT_deltaT;

///GYRO ext Computation

extern int GYRO_Counter;
extern int TC_Det_GYRO_Compute_Count;
extern double gyrodet_w[3];

// gps
extern unsigned int GPS_PPS_OBT;
extern unsigned int GPS_READ_OBT;


///Rate reduction routine
extern int BDOT_Reduction_Count, BDOTNormCount, BDOT_Norm_Count;
extern double BDOTnorm;
extern double TC_BDOT_Norm_Threshold;
extern double TC_BDOT_Det_Thresh;

///SunAcquisition_ModePreprocessing

extern int SunAcquisition2ThreeAxis_autotransit;
extern double SunAcq_Ang_Thres;
extern int SunAcq3ThreeAx_trsit_cnt;
extern int SunAcq2ThreeAx_trsit_cnt_thres;
extern int SunAcq2DetMode_counter;

extern int ThreeAxis2DetMode_counter, f_threeaxis2safe, f_battery_safemode;

/// Timer based sunlit/eclipse

extern float entrytime2eclipse, orbit_time;

/// Autotransit

extern int TC_AutoTransitAnyMode2Det;

/// Control Bytes

extern int CB_Detumbling_Mode;
extern int CB_Q_propagation;
extern int CB_DAD_quest;
extern int CB_QuestDataProcessing;
extern int CB_ErrorComputation;
extern int CB_ExtendedKalmanFilter;
extern int CB_LinearController;
extern int CB_Wheel_OverSpeed_TorqueCutOff;
extern int CB_DutyCycleGeneration;
extern int CB_AngularMomentumDumping;
extern int CB_SpeedBasedMomentumDumping;
extern int CB_Wheel_Dynamic_Friction;
extern int CB_Wheel_Spin_updown;
extern int CB_Wheel_Auto_Reconfiguration;
extern int CB_Two_RW_control;
extern int CB_Torquer_Polarity_Check;
extern int CB_MagFieldComp;
extern int CB_Sun_model;
extern int CB_ReferenceQuatComputation;
extern int CB_RefVectorGeneration;
extern int CB_RefRate_Computation;
extern int CB_Sl_Ecl_OnBrd_detection;
extern int CB_IMUDataProcessing;
extern int CB_SunSensorDataProcessing;
extern int CB_BDOT_Computation;

extern int f_RW_control, f_RW_nominal, RW_nominal_speed_cnt;

/// Power on

void rADCS_Pon_vars(void);
/// Function declarations
extern void rGH_generation(void);
extern void rMagFieldComp(void);
extern void rSun_Ephemeris(void);
extern void rReferenceQuatComputation(void);
extern void rRefVectorGeneration(void);
extern void rRefRate_Computation(void);
extern void rSl_Ecl_OnBrd_detection(void);

///Function declarations
extern void rGPS_TM_Extract(void);
extern void GPS_1_DATA(void);
extern void rnut_iau1980(double TTDBin, const double *fin);
extern void rast_args(double TTDBin);
extern void rSidereal(void);
extern void rxRot(double th);
extern void ryRot(double th);
extern void rzRot(double th);
extern void rTLEDataProcessing(void);
extern void rOrbitalElements_generation_GPS(void);
extern void rOrbit_Initialization(void);
extern void rJulian_Day(int year, int mon, int days, int hr, int minute, double sec);
extern void rECEFtoECItoECEF(void);
extern void rMatInv(double mat2inv[3][3]);
extern void rOrbit_Propagation(void);
extern void rOrbitalElements_computation(double Pos_ECI_in[3], double Vel_ECI_in[3], double Pos_ECEF_in[3]);
extern void rNEDtoECEF(void);
extern void rGPSDataProcessing(void);

extern void rSunSensorDataProcessing(void);
extern void rIMUDataProcessing(void);
extern void rSS_Read_Data(double *SS_Data_Addr,unsigned long int *ADC_Data_Addr);
double* rSunSensorVectorComp(double* SS_Data_Addr, struct SunSensor_Database *SS_2Exe_Addr);
double* rIMUDataCorrection(struct IMU_READ_DATA* IMU_DATA,struct IMU_Database* IMU_CORR);
extern double rTheta_Limit(double inter_theta);
extern void rBDOT_Computation(void);

extern void rQuestDataProcessing(void);
extern void rGPSTLEProcessing(void);

extern void rQ_Propagation(double q_prop[4], double w_prop[3]);

extern void rDAD_quest(void);

extern void rSuspended_ModePreprocessing(void);
extern void rDetumbling_ModePreprocessing_GYRO_Logic(void);
extern void rDetumbling_ModePreprocessing_BDOT_Logic(void);
extern void rSunAcquisition_ModePreprocessing(void);
extern void rThreeAxis_ModePreprocessing(void);
extern void rAnyMode2DetMode_transit(void);
extern void rSafeMode_Preprocessing(void);
extern void rScModeSelection(void);

///Function declaration
extern void rMatMul3x3(double mat331[3][3], double mat332[3][3]);
extern void rMatMul3x1(double mat33[3][3], double vec31[3]);
extern void rMatMul44x1(double mat44[4][4], double vec41[4]);
extern void rCross_Product(double u[3], double v[3]);
extern void rRM_to_Quat(double DCM2Q_in[3][3]);
extern void rQs_Normalization(double in_Quat_norm[4]);
extern void rVectorNorm(double vecnorm_in[3]);
extern void rQs_Normalization(double in_Quat_norm[4]);
extern void rMat_adjoint3(double in_mat_adj[3][3]);
extern void rRM_to_Quat(double DCM2Q_in[3][3]);
extern void rQuat_multiply(double in_quat1[4],double in_quat2[4]);
extern void rQs_Multiplication(double in_Quat1[4],double in_Quat2[4]);
extern void rMatMul43x1(double mat43[4][3], double vec31[4]);
extern void rMatMul34x1(double mat34[3][4], double vec41[4]);


extern void rWheel_Dither_Torque(void);
extern void rWheel_Spin_updown(void);
extern void rWheel_Auto_Reconfiguration(void);
extern void rLinearController(void);
extern void rWheel_Dynamic_Friction(void);
extern void rAngularMomentumDumping(void);
extern void rSpeedBasedMomentumDumping(void);
extern void rErrorComputation(void);
extern void rExtendedKalmanFilter1_p1(void);
extern void rExtendedKalmanFilter1_p2(void);

extern void rExtendedKalmanFilter2_p1(void);
extern void rExtendedKalmanFilter2_Prop(void);
extern void rExtendedKalmanFilter2_p2(void);
extern void rEKFDynamics(void);
extern void rEKF_dy_int(double kfq_dy[4], double kfw_dy[3], double kfv_dy[4]);

extern void rTwo_RW_control(void);
extern void rDutyCycleGeneration(void);
extern void rTorquer_Polarity_Check(void);


/// for dynamics

int tor_counter;

//for gain select
extern double TC_detumbling_rate_gain[3];
extern double TC_detumbling_bdot_gain[3];
extern double TC_AngMomDump_Thrsld;
extern double TC_momentum_dumping_gain;
extern double TC_comd_pitch_rate;
extern double DB_GyroLPF;
extern double DB_MagLPF;
#endif // ADCS_VARDECLARATIONS_H_INCLUDED






