#ifndef ADCS_CONSTANTS_H_INCLUDED
#define ADCS_CONSTANTS_H_INCLUDED

#define abs_f(a) (((a) < 0.0) ? (-1.0 * (a)) : (a))
#define abs_i(a) (((a) < 0) ? (-1 * (a)) : (a))
#define sign_f(a) (((a) < 0.0)? (-1.0) : (1.0))
#define sign_i(a) (((a) < 0)? (-1) : (1))

#define True 1
#define False 0
#define Enable 1
#define Disable 0
#define DIVIDE_BY_ZERO 0.0000000000000001
#define Set 1
#define Reset 0

#define Cur_Positive 1
#define Cur_Negative -1
#define No_Current 0


///Orbit Model
    const double c_twomu;
    const double c_tsince_min;
    const double c_OMEGAE;
    const double c_vkmpersec;
    const double c_radiusearthkm;
    const double c_auer;
    const double c_au;
    const double c_j2;
    const double c_Twopi;
    const double c_x2o3;
    const double c_xke;
    const double c_dividebyzerovalue;
    const double c_mu;
    const double c_invmu;
    const double c_Day_To_Seconds;
    const double c_Pi;
    const double c_xpdotp;
    const double c_ss;
    const double c_qzms2t;
    const double c_j3oj2;
    const double c_j4;
    const double c_temp4;
    const double c_min_per_day;
    const double c_D2R;
    const double c_R2D;
    const double c_AS2R;
    const double c_Mmax;
    const double c_oneminute;
    const double c_onesecond;

    const int c_I_four_cross_four[4][4];
    const double fc[5][5];

    const double nut[106][10];


    ///IGRF
    const int c_Nmax;
    const int c_Kmax;
    const double c_Pibytwo;
    const double c_a;
    const double c_sdmax;
    const double c_MaC;
    const double c_MiC;

    //Telemetry
    const double c_TM_Resol_DelThta;
    const double c_TM_Resol_w;
    const double c_TM_Resol_B;
    const double c_TM_RW_Resol;

    //IMU
    const unsigned int c_FFFF;
    extern const double c_Resol_B; // Resolution for the Magnetometer
    extern const double c_Resol_DelThta; // Resolution for Delta Theta


const double c_gval[104];
const double c_gsval[104];
const double c_hval[104];
const double c_hsval[104];

///Sun Model
const double c_L_Msun1;
const double c_L_Msun2;
const double c_Msun1;
const double c_Msun2;
const double c_L_Ecliptic1;
const double c_L_Ecliptic2;
const double c_Sun_Dis1;
const double c_Sun_Dis2;
const double c_Sun_Dis3;
const double c_Epsilon1;
const double c_Epsilon2;
const double c_KDset1;
const double c_KDset2;
const double c_KDset3;

///16 Imax constants of for all sensors (Main and Redundant cells) ( NOTE: constant = Overall_Imax / Individual_Imax)
const double c_Imax_RPD_Red;
const double c_Imax_RND_Red;
const double c_Imax_RPND_Red;
const double c_Imax_RNND_Red;
const double c_Imax_PP_Red;
const double c_Imax_PN_Red;
const double c_Imax_YP_Red;
const double c_Imax_YN_Red;

const double c_Imax_RPD_Main;
const double c_Imax_RND_Main;
const double c_Imax_RPND_Main;
const double c_Imax_RNND_Main;
const double c_Imax_PP_Main;
const double c_Imax_PN_Main;
const double c_Imax_YP_Main;
const double c_Imax_YN_Main;

///---------------------------------------------------------------------------------------------------------------------------
const double c_SSThrsld; ///Sunsensor threshold value
const double c_Sunlit_Thrsld; ///Threshold for sunlit detection
const double c_AngDev_SMtransit_thrsld; /// (30 degrees) Threshold for sun presence in negative pitch side
const double c_AngDev_SAMtransit_thrsld; /// (30 degrees) Threshold for sun presence in negative pitch side

///SunSensor's Misalignment Correction Matrices
const double c_misaln_cor125[3][3];
const double c_misaln_cor126[3][3];
const double c_misaln_cor325[3][3];
const double c_misaln_cor326[3][3];
const double c_misaln_cor345[3][3];
const double c_misaln_cor346[3][3];
const double c_misaln_cor145[3][3];
const double c_misaln_cor146[3][3];

///Sun Sensor to Body frame conversion for all the 8 quadrants
const double c_ss2b1256[3][3];
const double c_ss2b2356[3][3];
const double c_ss2b3456[3][3];
const double c_ss2b4156[3][3];

///Quest
const double c_wks_mag;
const double c_wkm_mag;
const double c_wks_sunmag;
const double c_wkm_sunmag;



/// Linear Controller
const double c_DPMMAX;
const double c_RPM2RADpS;
const double c_RADps2RPM;
const double TC_T_RW_MAX;

const double c_MOI_wh1, c_MOI_wh2, c_MOI_wh3, c_MOI_wh4, c_MOI_wh;

const double c_wh2B_mat_4RW[3][4];

const double c_wh2B_mat_1230[3][4];

const double c_wh2B_mat_1204[3][4];

const double c_wh2B_mat_1034[3][4];

const double c_wh2B_mat_0234[3][4];

const double c_wh2B_mat_1200[3][4];

const double c_wh2B_mat_1030[3][4];

const double c_wh2B_mat_1004[3][4];

const double c_wh2B_mat_0230[3][4];

const double c_wh2B_mat_0204[3][4];

const double c_wh2B_mat_0034[3][4];

const double c_B2wh_mat_4RW[4][3];


const double c_B2wh_mat_1230[4][3];

const double c_B2wh_mat_1204[4][3];

const double c_B2wh_mat_1034[4][3];


const double c_B2wh_mat_0234[4][3];

const double c_B2wh_mat_1200[4][3];

const double c_B2wh_mat_1030[4][3];

const double c_B2wh_mat_1004[4][3];

const double c_B2wh_mat_0230[4][3];

const double c_B2wh_mat_0204[4][3];

const double c_B2wh_mat_0034[4][3];

const double c_Tv_wof_1200[3][3],c_Tv_wof_1030[3][3],c_Tv_wof_1004[3][3],c_Tv_wof_0230[3][3],c_Tv_wof_0204[3][3],c_Tv_wof_0034[3][3];

const int c_DPM_Pol_LookUpTable[27][3];

///Kalman Filter
const double c_I_three_cross_three[3][3];
const double c_I_nine_cross_nine[9][9];
const double c_rk[9][9];
const double c_rk_T[9][9];

const double c_R_MDO_CB[3][3];

#endif // ADCS_CONSTANTS_H_INCLUDED






