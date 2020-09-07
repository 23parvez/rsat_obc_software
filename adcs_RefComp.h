#ifndef ADCS_REFCOMP_H_INCLUDED
#define ADCS_REFCOMP_H_INCLUDED

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
unsigned char Station_tracking_mode;

//Sun Model
extern double S_ECI[3], S_ECIn[3];
extern double L_Msun, Msun, L_Ecliptic, Sun_Dis, Epsilon;

// Ref quaternions
extern double SUN_ECI_mag, X_SVO2ECI_mag, Z_SVO2ECI_mag, Y_EPO2ECI_mag, X_EPO2ECI_mag, Z_SFAO2ECI_mag, X_SFAO2ECI_mag;
extern double X_SFDO2ECI_mag, Z_SFDO2ECI_mag;
extern double Y_SVO2ECI[3], Z_SVO2ECI[3], X_SVO2ECI[3], R_SVO2ECI[3][3], Q_SVO2ECI[4];
extern double X_MDO2ECI[3],Y_MDO2ECI[3],Z_MDO2ECI[3],R_MDO2ECI[3][3],Q_MDO2ECI[4];
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

//Ref Vector Generation
extern double Q_REF[4], Q_REF_conj[4], Q_StP2ECI[4], R_StP2ECI[3][3];
extern double Q_svn_off[4], Q_stn_off[4], Q_REF_GND[4];
extern double B_REF[3], S_REF[3], B_REFn[3], S_REFn[3];

// Ref Gyro

extern double w_REF[3], w_REF_prev[3];
extern double Q_REF_pres[4], Q_REF_prev[4], Q_REF_prev_conj[4];
extern double Q_REF_diff[4], QRD_vect_norm;
extern double Q_angle, Q_axis[3];

// Onboard Eclipse Algorithm
extern double theta1_se, theta2_se;
extern double psi_sl_ecl;
extern int elapsed_running_timer;
extern int f_Sunlit_Presence_orbit, f_Sunlit_Presence_timer, f_Sunlit_Presence_sensor;
extern int Sunlit_presence_timer, Eclipse_presence_timer;
extern int f_station_tracking_enabled,f_station_tracking_enabled_pre;
extern double rsun[3], rsat[3], magrsun;

extern double tempse;
extern double bsqrd;
extern double asqrd;
extern double adotb;
extern double distsqrd;
extern double tmin;

//Ref Vector Generation
extern double Q_REF[4], Q_StP2ECI[4], R_StP2ECI[3][3];
extern double B_REF[3], S_REF[3], B_REFn[3], S_REFn[3];



// Function declarations

extern void rGH_generation(void);
extern void rMagFieldComp(void);
extern void rSun_Ephemeris(void);
extern void rReferenceQuatComputation(void);
extern void rRefVectorGeneration(void);
extern void rRefRate_Computation(void);
extern void rSl_Ecl_OnBrd_detection(void);


#endif // ADCS_REFCOMP_H_INCLUDED

