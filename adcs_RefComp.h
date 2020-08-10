#ifndef ADCS_REFCOMP_H_INCLUDED
#define ADCS_REFCOMP_H_INCLUDED

/// IGRF model
int i_rfc, j_rfc;

double temp_magad;
double BNorth,BEast,BNorth_old,BDown,rot_ang,A1_magad,A2_magad,A3_magad,B_NED[3],B_ECI[3],B_ECIn[3],B_ECItesla[3];
double co_dec;
int fN_MAGAD,gN_MAGAD,gmm;
double st_magad,ct_magad,sl_magad[14],cl_magad[14];
double one_magad,two_magad,sd_magad,p_magad[106],q_magad[106];
int I_MFC,K_MAGAD,I_MAGAD,J_MAGAD,M_MAGAD,N_MAGAD;
double Cond1;
double B_ECEF[3];
double A_R, Alti_Mod, Rho, B_Rho, cd_magad, old_cos, s_magad;
unsigned char m_gh, n_gh, i_gh, j_gh;
double g[15][16], h[15][16];
double REF_FRAME_DCM[3][3];
int DeltaT_MFC, DeltaT_Updated;

///Sun Model
double S_ECI[3], S_ECIn[3];
double L_Msun, Msun, L_Ecliptic, Sun_Dis, Epsilon;

/// Ref quaternions
double SUN_ECI_mag, X_SVO2ECI_mag, Z_SVO2ECI_mag, Y_EPO2ECI_mag, X_EPO2ECI_mag, Z_SFAO2ECI_mag, X_SFAO2ECI_mag;
double X_SFDO2ECI_mag, Z_SFDO2ECI_mag;
double Y_SVO2ECI[3], Z_SVO2ECI[3], X_SVO2ECI[3], R_SVO2ECI[3][3], Q_SVO2ECI[4];

double X_MDO2ECI[3],Y_MDO2ECI[3],Z_MDO2ECI[3],R_MDO2ECI[3][3],Q_MDO2ECI[4],R_MDO_CB[3][3];

double Pos_ECIn[3], rdotrst, SAT_ANGLE_STAT;
double STATION_ECEF[3], STATION_ECI[3], STATION_vector[3], STATION_ECIn[3];
double X_SPO2ECI[3], Y_SPO2ECI[3], Z_SPO2ECI[3];

double X_SPO2ECI_mag, Y_SPO2ECI_mag;
double R_SPO2ECI[3][3], Q_SPO2ECI[4];

double Z_EPO2ECI[3], Y_EPO2ECI[3], X_EPO2ECI[3], R_EPO2ECI[3][3], Q_EPO2ECI[4];
double Y_SFAO2ECI[3], Z_SFAO2ECI[3], X_SFAO2ECI[3], R_SFAO2ECI[3][3], Q_SFAO2ECI[4];
double Y_SFDO2ECI[3], Z_SFDO2ECI[3], X_SFDO2ECI[3], R_SFDO2ECI[3][3], Q_SFDO2ECI[4];

///Ref Vector Generation
double Q_REF[4], Q_StP2ECI[4], R_StP2ECI[3][3];
double Q_svn_off[4], Q_stn_off[4], Q_REF_GND[4];
double B_REF[3], S_REF[3], B_REFn[3], S_REFn[3];

/// Ref Gyro

double w_REF[3], w_REF_prev[3];
double Q_REF_pres[4], Q_REF_prev[4], Q_REF_prev_conj[4];
double Q_REF_diff[4], QRD_vect_norm;
double Q_angle, Q_axis[3];

/// Onboard Eclipse Algorithm
double theta1_se, theta2_se;
double psi_sl_ecl, elapsed_running_timer;
int f_Sunlit_Presence_orbit, f_Sunlit_Presence_timer, f_Sunlit_Presence_sensor;
int Sunlit_presence_timer, Eclipse_presence_timer;

int f_station_tracking_enabled,f_station_tracking_enabled_pre;


double rsun[3], rsat[3], magrsun, tempse, bsqrd, asqrd, adotb, distsqrd, tmin;


/// Function declarations
void rGH_generation(void);
void rMagFieldComp(void);
void rSun_Ephemeris(void);
void rReferenceQuatComputation(void);
void rRefVectorGeneration(void);
void rRefRate_Computation(void);


#endif // ADCS_REFCOMP_H_INCLUDED

