#ifndef ADCS_CONSTANTS_H_INCLUDED
#define ADCS_CONSTANTS_H_INCLUDED

//Orbit Model
extern const  double c_twomu;
extern const  double c_tsince_min;
extern const  double c_OMEGAE;
extern const  double c_vkmpersec;
extern const  double c_radiusearthkm;
extern const  double c_polarradiuskm;
extern const  double c_ecc_ellip;
extern const  double c_auer;
extern const  double c_au;
extern const  double c_j2;
extern const  double c_Twopi;
extern const  double c_x2o3;
extern const  double c_xke;
extern const  double c_dividebyzerovalue;
extern const  double c_mu;
extern const  double c_invmu;
extern const  double c_Day_To_Seconds;
extern const  double c_Pi;
extern const  double c_xpdotp;
extern const  double c_ss;
extern const  double c_qzms2t;
extern const  double c_j3oj2;
extern const  double c_j4;
extern const  double c_temp4;
extern const  double c_min_per_day;
extern const  double c_D2R;
extern const  double c_R2D;
extern const  double c_AS2R;
extern const  double c_Mmax;
extern const  int c_oneminute;
extern const  int c_onesecond;
extern const  double c_halfhour;

extern const  double fc[5][5];

extern const  double nut[106][10];


//IGRF
extern const  int c_Nmax;
extern const  int c_Kmax;
extern const  double c_Pibytwo;
extern const  double c_a;
extern const  double c_sdmax;
extern const  double c_MaC;
extern const  double c_MiC;


extern const  double c_gval[104];
extern const  double c_gsval[104];
extern const  double c_hval[104];
extern const  double c_hsval[104];

//Sun Model
extern const  double c_L_Msun1;
extern const  double c_L_Msun2;
extern const  double c_Msun1;
extern const  double c_Msun2;
extern const  double c_L_Ecliptic1;
extern const  double c_L_Ecliptic2;
extern const  double c_Sun_Dis1;
extern const  double c_Sun_Dis2;
extern const  double c_Sun_Dis3;
extern const  double c_Epsilon1;
extern const  double c_Epsilon2;
extern const  double c_KDset1;
extern const  double c_KDset2;
extern const  double c_KDset3;

extern const  int c_DPM_Pol_LookUpTable[27][3];

//Kalman Filter
extern const  double c_I_three_cross_three[3][3];
extern const  double c_I_nine_cross_nine[9][9];
extern const  double c_rk[9][9];
extern const  double c_rk_T[9][9];

//16 Imax constants of for all sensors (Main and Redundant cells) ( NOTE: constant = Overall_Imax / Individual_Imax)
extern const  double c_Imax_RPD_Red;
extern const  double c_Imax_RND_Red;
extern const  double c_Imax_RPND_Red;
extern const  double c_Imax_RNND_Red;
extern const  double c_Imax_PP_Red;
extern const  double c_Imax_PN_Red;
extern const  double c_Imax_YP_Red;
extern const  double c_Imax_YN_Red;

extern const  double c_Imax_RPD_Main;
extern const  double c_Imax_RND_Main;
extern const  double c_Imax_RPND_Main;
extern const  double c_Imax_RNND_Main;
extern const  double c_Imax_PP_Main;
extern const  double c_Imax_PN_Main;
extern const  double c_Imax_YP_Main;
extern const  double c_Imax_YN_Main;

//---------------------------------------------------------------------------------------------------------------------------
extern const  double c_SSThrsld; //Sunsensor threshold value
extern const  double c_Sunlit_Thrsld; //Threshold for sunlit detection
extern const  double c_AngDev_SMtransit_thrsld; // (30 degrees) Threshold for sun presence in negative pitch side
extern const  double c_AngDev_SAMtransit_thrsld; // (30 degrees) Threshold for sun presence in negative pitch side

//SunSensor's Misalignment Correction Matrices
extern const  double c_misaln_cor125[3][3];
extern const  double c_misaln_cor126[3][3];
extern const  double c_misaln_cor325[3][3];
extern const  double c_misaln_cor326[3][3];
extern const  double c_misaln_cor345[3][3];
extern const  double c_misaln_cor346[3][3];
extern const  double c_misaln_cor145[3][3];
extern const  double c_misaln_cor146[3][3];

//Sun Sensor to Body frame conversion for all the 8 quadrants
extern const  double c_ss2b1256[3][3];
extern const  double c_ss2b2356[3][3];
extern const  double c_ss2b3456[3][3];
extern const  double c_ss2b4156[3][3];

//Quest
extern const  double c_wks_mag;
extern const  double c_wkm_mag;
extern const  double c_wks_sunmag;
extern const  double c_wkm_sunmag;

// Linear Controller
extern const  double c_MOI_wh;
extern const  double c_DPMMAX;
extern const  double c_RPM2RADpS;
extern const  double c_RADps2RPM;
extern const  double TC_T_RW_MAX;
extern const  double TC_Hmin;

extern const  double c_wh2B_mat_4RW[3][4];

extern const  double c_wh2B_mat_1230[3][4];

extern const  double c_wh2B_mat_1204[3][4];

extern const  double c_wh2B_mat_1034[3][4];

extern const  double c_wh2B_mat_0234[3][4];

extern const  double c_B2wh_mat_4RW[4][3];


extern const  double c_B2wh_mat_1230[4][3];

extern const  double c_B2wh_mat_1204[4][3];

extern const  double c_B2wh_mat_1034[4][3];


extern const  double c_B2wh_mat_0234[4][3];


//ADandEstimation
extern const  int c_I_four_cross_four[4][4];
extern const  unsigned int c_FFFF;
extern const  double c_Resol_DelThta;
extern const  double c_Resol_B;
extern const  double c_TM_Resol_DelThta;
extern const  double c_TM_Resol_w;
extern const  double c_TM_Resol_B;
extern const  double c_TM_RW_Resol;

extern const  double c_MagBias_act_LUT[27][3];

extern double c_I_MAT[3][3], c_I_MAT_Inv[3][3];
extern double c_MOI_wh_mat_Inv[4][4];
extern double c_MOI_wh_mat[4][4];


#endif // ADCS_CONSTANTS_H_INCLUDED






