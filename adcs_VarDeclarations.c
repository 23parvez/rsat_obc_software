#include "adcs_VarDeclarations.h"

int i_pini, j_pini;

double B_BODYtesla[3], Bsq, B_BODY[3], B_BODYn[3];
double magRoll_angle, magPitch_angle, magYaw_angle;
double w_BODY[3], w_BODYdeg[3], Thta_rawdata[3], w_BODYnorm;
double DelThta_rawdata[3], w_ABC[3], w_BiasCor[3], w_ASC[3], w_AMSC[3],w_RPY[3],w_LPF[3];
double B_IMU[3], B_BiasCor[3], B_ASC[3], B_AMSC[3],B_RPY[3],B_LPF[3];
double w_BODY_IMU1[3], w_BODY_IMU2[3], B_BODY_IMU1[3], B_BODY_IMU2[3];
double IMU_prcd_data[6];
double* IMU_prcd_data_ptr;
double IMU_Sen2Bdy[3][3];
double B_BODY_LUT[3];

int PolCheck_LUT, PolChec_LUT_res;
double MagBias_residue_LUT[27][3], MagBias_act_LUT[27][3];

int DelThta_cat_x; //Delta Theta data for x axis after concatenating LSB and MSB bits
int DelThta_cat_y; //Delta Theta data for y axis after concatenating LSB and MSB bits
int DelThta_cat_z; //Delta Theta data for z axis after concatenating LSB and MSB bits

int B_rawdata[3]; // Magnetic field raw data
int inter_imu_i,inter_imu_j;
double Thta_BODY_IMU1[3], Thta_BODY_IMU2[3];

unsigned int imu1_db_checksum_obc;
unsigned int imu2_db_checksum_obc;

double TC_wAD_BODYmaxThRoll, TC_wAD_BODYmaxThPitch, TC_wAD_BODYmaxThYaw, TC_wAD_BODYminThRoll, TC_wAD_BODYminThPitch, TC_wAD_BODYminThYaw;
int wAD_updatecount, TC_wAD_updateTimeThresh;


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
double Z_EPO2ECI[3], Y_EPO2ECI[3], X_EPO2ECI[3], R_EPO2ECI[3][3], Q_EPO2ECI[4];
double Y_SFAO2ECI[3], Z_SFAO2ECI[3], X_SFAO2ECI[3], R_SFAO2ECI[3][3], Q_SFAO2ECI[4];
double Y_SFDO2ECI[3], Z_SFDO2ECI[3], X_SFDO2ECI[3], R_SFDO2ECI[3][3], Q_SFDO2ECI[4];

double rdotrst, SAT_ANGLE_STAT;
double STATION_ECEF[3], STATION_ECI[3], STATION_vector[3], STATION_ECIn[3];
double X_SPO2ECI[3], Y_SPO2ECI[3], Z_SPO2ECI[3];

double TC_eesqrd;
double TC_long_station, TC_lat_station;

double X_SPO2ECI_mag, Y_SPO2ECI_mag;
double R_SPO2ECI[3][3], Q_SPO2ECI[4];

///Ref Vector Generation
double Q_REF[4],Q_REF_conj[4], Q_StP2ECI[4], R_StP2ECI[3][3];
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

int f_station_tracking_enabled,f_station_tracking_enabled_pre;
double rsun[3], rsat[3], magrsun;

double tempse  = 0.0;
double bsqrd  = 0.0;
double asqrd  = 0.0;
double adotb  = 0.0;
double distsqrd  = 0.0;
double tmin = 0.0;

///To be deleted
int TC_EarthPointingFrame;
int TC_SunEarthVaryingFrame;
int TC_SunFixedAscendingFrame;
int TC_SunFixedDescendingFrame;
int TC_StationTracking;

///----------AD and Estimation---------------

/// Quest Data Processing
int sc_qst; ///sample count for magAD data processing(MC)
int dc_qst; ///data count for magAD data processing (sec) NOTE: NOT MC
int wc_qst; ///wait period count for magAD data processing (sec) NOTE: NOT MC
int mat_mag_DataCounter, mat_sm_DataCounter; ///data counter for magAD and sunmagAD to track the filling matrices
int TC_wp_QDP; ///Telecommand for Wait period selection in QDP
double NMB_mag[3][24]; ///Measurement Matrix (Magnetic field) for magAD
double NRB_mag[3][24]; ///Reference Matrix (Magnetic field) for magAD
double NMB_sunmag[3][8]; ///Measurement Matrix (Magnetic field) for sunmagAD
double NRB_sunmag[3][8]; ///Reference Matrix (Magnetic field) for sunmagAD
double NMS[3][8]; ///Measurement Matrix (sun sensor) for sunmagAD
double NRS[3][8];  ///Reference Matrix (sun model) for sunmagAD
int f_DataSort_MAG,f_DataSort_SUNMAG; ///Flags that are raised to tell data matrices are ready for Quaternion computation in QUEST
int i_QDP;
int CB_Q_propagation, TC_enQuest_update, Quest_update_available, w_q_update_satisfy;

int CB_DAD_quest;
double B_DAD[3][3], Bt_DAD[3][3];
double NRBt_mag[24][3], NRSt[8][3];
double NRBt_sunmag[8][3];
double wkm, wks;
double S_DAD[3][3], St_DAD[3][3], S_sqr_DAD[3][3];
double Z_DAD[3], Zt_DAD[3];
double sigma_DAD, delta_DAD, a_DAD, b_DAD,c_DAD,d_DAD, k_DAD;

double root_DAD, func_DAD, Dfunc_DAD, func_prev_DAD;
double alpha_DAD, beta_DAD, gamma_DAD;
double X_DAD[3];

double TC_magMin_angle, TC_magMax_angle;
double Qquest_update[4];
double Q_quest_DAD[4];
int I_DAD, J_DAD, K_DAD;
double NMB_NRBt[3][3], NMS_NRSt[3][3];
double Zt_Z_DAD, Zt_S_Z_DAD, Zt_Ssqr_Z_DAD;
double alpha_I_DAD[3][3], beta_S_DAD[3][3], sumX_DAD[3][3];
int TC_SunMagAD;
double trace_S_DAD, adj_S_DAD[3][3];

/// Qgyro

double Del_Y_theta, Del_R_theta, Del_P_theta; ///Small angle obtained by integrating Gyro data over a periof of time (mc/MC)
double Del_Q[4]; ///Delta Qs
double q_prop_out[4],Qprop_prev[4];
double Qbody[4];

double W_cross[3][3];
double big_omega[4][4];
double Delta_by2;
double W_norm;
double Delta;
double cos_of_W_norm_and_Delta_by2;
double cos_of_W_norm_Delta_by2_with_I[4][4];
double sin_of_W_norm_and_Delta_by2;
double sin_of_W_norm_Delta_by2_and_W_norm;
double sin_of_W_norm_Delta_by2_W_norm_and_big_omega[4][4];
double cos_and_sin[4][4];

///New variables

int GPSDataReady_NA_count;
int TC_GPS2TLE_Switch;
int Present_OBT;
int OBT_at_TLE_epoch;
int Delta_TLE;
int TC_GPSvalidity_Threshold;

//GPS
unsigned char* GPS_TM_Buffer_Addr_USC;
unsigned long int GPS_Data_Read_Status;
unsigned long int GPS_Buffer_Data[106];
unsigned long int GPS_RCVD_DATA[106];
unsigned char GPS_obc_checkum;
unsigned int f_GPS_Valid_Data;

///Julian Day
double jd_time, tut;
double pps_deltaT, Julian_day;

///ast_args
double tt[4], f[5];

///main
int i_god, j_god;
int minorcyclecount, majorcyclecount;
int k, i, j, ktr, TLE_Select;

///rOrbitalElements_generation_GPS
int GPS_Select, GPSDataReady, TC_GPS_pulse_duration;
int i_jday, lmonth[13], year_GPS, UTC_mon_GPS, tempdays, Numofdays, UTC_day_GPS, UTC_hr_GPS, UTC_min_GPS;

double epochdays_GPS, UTC_sec_GPS, PosX_ECEF_GPS, PosY_ECEF_GPS, PosZ_ECEF_GPS, Pos_ECEF_GPS[3];
double VelX_ECEF_GPS, VelY_ECEF_GPS, VelZ_ECEF_GPS, Vel_ECEF_GPS[3], Vel_ECI_temp[3], Vel_ECI_GPS[3], Pos_ECI_GPS[3];
double rpef[3], vpef[3], wcrecef[3], e_vec[3], angmomentumvec[3];
int Epochyear_GPS;

double radialdistance, radialdistance_ecef, velmag, r_delta, velmagsq, rdtv, ecc_temp1, ecc_temp2;
double ecc_temp3, ecc_temp4, ecc_temp5, ecc_temp6, ecc_temp7, ecc_GPS, semi_den, semimajoraxis_GPS, Alti_GPS;
double angmomentumvecmag, delta_hmag, invangmomentumvecmag;
double inclination_GPS, sinlongacnode, coslongacnode, nodeo_GPS, e_vecdtr, tempta, trueanomoly_GPS, Ndte_vec, N_ecc;
double temparg, argpo_GPS, omecc, sine, cose, eccanomaly_GPS, mo_GPS, Orbit_Period, no_GPS, longitude_GPS, latitude_GPS;

///Orbital Elements generation TLE
double sec_TLE_tc, no_TLE_tc, ecc_TLE_tc, trueanomoly_TLE_tc, epochdays_TLE_tc;
int minute_TLE_tc, hr_TLE_tc, day_TLE_tc, mon_TLE_tc, year_TLE_tc, Epochyear_TLE_tc, TC_TLE_Elements_select;
int epochyr, dayofyr, inttemp;
double epochdays, temp_jday, ibexp_TLE_tc, bstar_TLE_tc, inclination_TLE_tc, nodeo_TLE_tc, argpo_TLE_tc;
double mo_TLE_tc, ibexp_tc, bstar_tc, inclo_tc, nodeo_tc, argpo_tc, mo_tc;

///Orbit Initialization and Propagation
double Tsince_GPS, epochdays_sel, inclination_sel, nodeo_sel, trueanomoly_sel, mo_sel, argpo_sel;
double ecc_sel, no_sel, ibexp_sel, bstar_sel, sec_sel;
int year_sel, mon_sel, days_sel, hr_sel, minute_sel, epochyr_sel;
int GPS_Elements_Available, TLE_Data_Available;
double rtom, ao_sfour, pow_psisq, psetasq, intermediate, am_den, temp_temp, temp1, wcreci[3];
double Tsince, Tsince_TLE;
double Tsince_TLE_tc, Day_Of_Year_DeltaT, Delta_T, Orbit_Period_Comp, wo, inclo, nodeo, argpo, mo, ecco, no, ibexp, bstar;
int CB_OrbitModel, OrbitModel_Start;
char chksum_tle;

double con41, cc1, cc4, cc5, d2, d3, d4, delmo, eta, argpdot, omgcof, sinmao, aycof, t2cof, t3cof;

double t4cof, t5cof, x7thm1, mdot, nodedot, xmcof, nodecf, cc3, var;

double uuu, temp_prop, var1;


double eccsq, omeosq, rteosq, cosio, cosio2;

double ak, d1,  del, adel, ao, sinio, po, con42, posq, rp;

double sfour, qzms24;

double pinvsq, tsi, etasq, eeta, psisq,  coef, cc2, coef1,qzms24_temp;

double cosio4, temp1_or_init, temp2, temp3, temp_or_init, temp_om;

double xhdot1, xlcof;

double cc1sq, ak_temp, ao_temp, delmo_temp;

double x1mth2, perigee;

double xmdf, argpdf, nodedf, mm, t2, tempa, tempe, delomg, delm, t3, t4, nm, em, inclm, ecose, am, argpm, nodem, xlm, sinim, cosim;
double ep, xl, sinip, cosip, xincp, argpp, nodep, mp, axnl, aynl, esine, r1;
double betal, sinu, cosu, suu, sin2u, cos2u, mrt, xnode, xinc, mvt, rvdot, emsq, sinsu, cossu, snod, cnod, sini, cosi;
double xmx, xmy, ux, uy, uz, vx, vy, vz,  xsatx_eci, xsaty_eci, xsatz_eci, vsatx_eci, vsaty_eci, vsatz_eci;
double templ, am_temp, nm_temp, mm_temp, u_temp, el2, pl, rl, rdotl, rvdotl;
double sinkp, coskp, eol, etempe;
double del_temp;
double Pos_ECI[3], Vel_ECI[3], Pos_ECEF[3], Vel_ECEF_temp[3], Vel_ECEF[3];
double Pos_ECIn[3], Vel_ECIn[3], Pos_ECEFn[3], Vel_ECEFn[3];

///Orbital elements computation
double semimajoraxis, ecc, Alti, inclination_temp, inclination, RAAN, trueanomoly, argofperigee, eccanomaly, ecc_r;
double longitude, latitude_temp, latitude, longitude_tan_num, longitude_tan_den;
double xa_gcgd, mua_gcgd,ra_gcgd,l_gcgd,dlambda_gcgd,h_gcgd,den_gcgd,rhoa_gcgd,dmu_gcgd,gd_gcgd;

///Ecef to ECI to ecef
double UT1, UTC, TC_delUT1, TAI, JDTDT, M_quad, sine1, sine2, TDB, TDT, TTDB, JDTDB, TTDB2, TTDB3, zeta, z, theta;
double pre_temp[3][3], precession[3][3], eps, dpsi, nut_temp[3][3], xin_temp, nutation[3][3], tut1, gmst0, gmst_, gast;
double sidereal[3][3], TC_xp, TC_yp, polarmotion[3][3], nut_sid_temp[3][3], ECEFtoECI[3][3], ECItoECEF[3][3],ang;
double TC_delAT, TTDT, deps;

// gps
unsigned int GPS_PPS_OBT;
unsigned int GPS_READ_OBT;

///NED to ECEF
double NEDtoECEF[3][3];

///16 Cells' output (Main and Redundant)
double SS_M1, SS_M2, SS_M3, SS_M4, SS_M5, SS_M6, SS_M7, SS_M8;

double *SS_prcd_data_ptr;
double SB_MAIN[3], SB_RED[3], SS_prcd_data[3];
///6 Active sensors' output after multiplying by constants

double SC1, SC2, SC3, SC4, SC5, SC6, SC7, SC8;

///6 Cells' Imax Factors
double SC1ImaxF, SC2ImaxF, SC3ImaxF, SC4ImaxF, SC5ImaxF, SC6ImaxF;

int TC_SS_Main_Cells_Sel, Panel_Deployment;


double comb1, comb2, temp11, temp12;

double ele1, az1, ele2; ///Elevation and azimuth angles

double sun_sf[3],S_BODY[3], S_BODYn[3]; ///SunSensor data in sensor frame

double ss2b[3][3];

double Ang_Deviation; ///arccos(dot(S_BODY,[0,-1,0]))

int f_Sunlit_Presence, f_Sunlit_Presence_previous;
int f_aft_statn_wait,aft_statn_cnt;

double SS_Data[16];
int inter_sunsensor_i,inter_sunsensor_j;

int i_MatEq, j_MatEq;
int Sunlit_presence_timer, Eclipse_presence_timer;

int SunNPP_SMtransit_counter;
int SunNPP_SMtransit_count_limit;
int SunNPP_SMtransit;

double AngDev_SAMtransit_thrsld;
int SunNPP_SAMtransit_counter;
int SunNPP_SAMtransit_count_limit;
int SunNPP_SAMtransit;
int sun_quadrant;
double Roll_ang_err, Yaw_ang_err;

//Checksum
unsigned long int ss_main_db_checksum_obc;
unsigned long int ss_redundant_db_checksum_obc;

///BDOT Computation

int BDOT_Counter;
int Det_BDOT_MC_Count;
int TC_Det_Bprev_Count;
int TC_BDOT_Compute_Count;
double BDOT[3],Bpresent[3];
double Bprev[3];

///Det gyro

int TC_Det_GYRO_Compute_Count, GYRO_Counter;
double gyrodet_w[3], gyrodet_B[3];


///CommonRoutines.h

int i_comr, j_comr, k_comr;
///Rotation matrices
double Rx[3][3], Ry[3][3], Rz[3][3];

///mat inverse
double determinant, Invmatout33[3][3];

///mat multiplication
double Matout31[3], Matout33[3][3], Matout441[4];



///Cross product
double Cross_Product[3];

///Vector Normalization
double Norm_out[3], vecnorm_mag;

double out_Quat_norm[4]; ///Global variable (Normalized Quaternion)
double mat_mult[3][3]; ///Global variable (matrix multipication of 2 matrices[3][3])
double mat_adj[3][3]; ///Global variable (Adjoint of matrix[3][3])
double DCM2Q_out[4]; ///Global variable (RM to Quaternion Conversion)

double Matout341[3];

double Matout431[4];

double out_Quat_mult[4]; ///Global Variable of the Out_Quat from Q Multiplication Routine

///Dutycycle Generation
int DutyCycleGenEnable, TorquerPolaritySetFlag;
double MR, MP, MY;
int Roll_MTR_Pol_Reversal, Pitch_MTR_Pol_Reversal, Yaw_MTR_Pol_Reversal;
int DPM_Polarity[3], Ton[3], Toff[3], DPM_Pol_prev[3];
int Roll_MTREnable, Pitch_MTREnable, Yaw_MTREnable;
int MTR_ActuationCycle;
double DPM[3];
double TorquerDutyCycle[3];
///double DutyCycle_Roll;
///double DutyCycle_Pitch;
///double DutyCycle_Yaw;
float ActuationCycle;

///Linear controller
int i_lict, j_lict;
double TC_KR[3];
double TC_KP[3];
double TC_wh_speed_thres;
double Qerror[4];
double RWSpeed[4], v0_rad_sec[4];
double H_wh[4], HB[3];
int CB_Wheel_OverSpeed_TorqueCutOff, DFriction_Compensation;
double B2wh_mat[4][3], wh2B_mat[3][4];
double speedDFCch;
int TachoHyst[4], DFCCount[4], FirstSpeed[4];
double DFCGainHigh, LPFK1HighSL, LPFK2HighSL, DFCGainLow, LPFK1LowSL, LPFK2LowSL;
double ExWhMom[4], DFCGainSL[4], LPFK1SL[4], LPFK2SL[4], WhMom0[4], ActWhMom[4], LossWhMom[4], LossWhMomFilt[4], DFCTorq[4];
int DFCCountLimSL[4], DFCcountHigh, DFCcountLow;
int wheel_spin_logic, spin_up_avg_count, spin_up_avg_count_2;
double del_v0[4],  del_v0a, T_RW_spin[4];
int wheel_index[4], wheel_index_ARCsum, Wheel_Config, RW_ARC_Logic, RW_ARC_Count, TC_RW_ARC_Count_thres, count_arc_w0, TC_ARC_Time_Cycle, count_arc_w1, count_arc_w2, count_arc_w3;
double pres_exp_whsp_ch[4], exp_whsp_ch[4], ch_obs_whsp[4], prev_obs_whsp_ch[4], diff_obs_exp_ch[4];
double TC_ARC_RPM_Thres;

float TC_RW_Nominal[4];

//const double c_MOI_wh1, c_MOI_wh2, c_MOI_wh3, c_MOI_wh4, c_MOI_wh;

///Angular Momentum Dumping
int f_Momentum_Dumping;
int dumping_on;
double delta_HB[3];
double TC_Hmin;
double TC_Hmax;
double TC_MDk;

///Speed based Momentum Dumping
double TC_max_whspeed, TC_min_whspeed, TC_SpeedDumpLimit, MOI_wh;
int TC_SpeedDumpTime;
int check_dump_wh[4];
double H_retn;
int TC_SpeedAngularMomemtumDumping, wh_sdump_start[4], speed_based_torquer_control;
double TB_sMD[3], u_parl[3], Tc[3], T_RW[4], T_RW_sdump[4], T_RWB[3];
double del_Vw[4];
double T_RWBn[3], tau_ms, min_TW;
double u_perp[3];
double Pse_Inv_Dist_Mat[4][3];

///Two RW control

double Twof_veh[3][3], Tv_wof[3][3], Tcv_temp[3], Tcv[3], Tdeficit_temp1[3], Tdeficit_temp2[3], Tdeficit[3], Tdefnorm;
double alpha_2RW_temp[3], alpha_2RW_tempnorm, alpha_2RW, B_Tef_ANG, mu_m_temp[3], mu_m[3];
double Tdes[3], T_MT_2RW[3], Trwa_mt_comp[3], Trwa[4], T_SD_RW[4], Tmm_actual[3];


///Kalman Filter
int i_kf, j_kf, k_kf;
double sigma_v;
double sigma_u;
double sigma_m;
double Qk_temp1;
double Qk_temp2;
double Qk_temp3;
double Qk_temp4;
double qk_DCM[3][3];
double ref_cross[3][3];
double Hk[3][9];
double pk_HkT[9][3];
double pk_plus_one[9][9];
double phi_k_pk_plus[9][9];
double phi_k_pk_plus_phi_k_T[9][9];
double rk_Qk[9][9];
double rk_Qk_rk_T[9][9];
double Qk[9][9];
double  phi_k_T[9][9];
double phi_k[9][9];
double F_k1[3][3], mod_of_w_k, jw_cross[3][3], F_k4_temp1[3][3], F_k4_temp2[3][3], F_k4[3][3], F_k[9][9];
double phi_12[3][3];
double phi_12_temp2[3][3];
double Result_of_sine_Delta_cube;
double Result_of_sine_Delta;
double phi_12_temp1[3][3];
double phi_11[3][3];
double phi_11_temp4[3][3];
double phi_11_temp3[3][3];
double cos_of_Wk1_Delta_by_Wk1_square;
double Q_quest_DAD_den;
double mod_of_Wk1_square;
double mod_of_Wk1_cube;
double cos_of_Wk1_Delta;
double phi_11_temp2[3][3];
double phi_11_temp1[3][3];
double Wk1_one[3][3];
double sine_of_Wk1_Delta_by_Wk1;
double sine_of_Wk1_Delta;
double mod_of_Wk1;
double pk_plus[9][9];
double Kk_Hk_with_minus_one[9][9];
double Kk_Hk[9][9];
double qk_plus[4], qk_plus_conj[4];
double qk_plus_temp[4];
double Yk[3];
double Xk[9];
double q_by_two[4][3];
double Xk_plus_temp[3];
double Xk_plus[9];
double Yk_minus_hk_Kk[9];
double Yk_minus_hk[3];
double Kk_matrix[9][9];
double K_temp[3][3];
double pk_HkT_Hk_Rk[3][3];
double pk_HkT_Hk[3][3];
double pk[9][9];
double Rk[3][3];
double Hk_temp[9][3];
double hk[3];
double Wk1[3][3];
double one_by_2;
double one_by_3;
double phi_12_temp3[3][3];
double qk_minus[4], hk_xk_minus[3];
int TC_KalmanFilter_ENABLE;
double sun_noise;
double mag_noise;

// Extended Kalman Filter 2

int ecl_ekf, kfmeas, tor_counterk;

double T_RW_NETk[3], T_MTk[3], Mk[3], T_MTk[3], T_NETk[3], w_k[3], RWSpeedk[4];
double b_k[3], lpfx_magbk, lpfy_magbk, lpfz_magbk, lpfb_k[3], lpfb_k_prev[3];
double pbk, qqbk;

double kfk1[4], kfq_dot[4], kfg1[3], kfw_dot[3], kfv1[4], kfv_dot[4], kfq01[4], kfw01[3], kfv01[4];
double kfk2[4], kfg2[3], kfv2[4], kfq02[4], kfw02[3], kfv02[4];
double kfk3[4], kfg3[3], kfv3[4], kfq03[4], kfw03[3], kfv03[4];
double kfk4[4], kfg4[3], kfv4[4];

double kfom[4][4], kfq_dot[4], kfv_dot[4], kfT_RW_NET_temp[4], kfT_RW_NET[3], kfH_wh_ind[4], kfH_wh2body[3], kfI_MAT_w0[3], kfI_MAT_w0_HW2B[3], kfT_NET[3], kfw0crossI_MAT_w0_HW2B[3], kfw_dot_temp[3], kfw_dot[3];
int f_EKF2_prop_en,f_EKF2_mag_bias_dis;
double I_MAT[3][3], I_MAT_Inv[3][3];
double MOI_wh_mat_Inv[4][4];
double MOI_wh_mat[4][4];

//int ThreeAxis_ModePreprocessing;

///Panel deployment
int Not_Deployed;
int All_Deployed;
int PosR_Deployed;
int NegR_Deployed;

/// Detumbling Mode
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

int BDOT_Threshold_Count;
int TorquerPolaritySetFlag;
//int SunAcquisition_ModePreprocessing;
double TC_Bdot_Gain[3];


///rDetumbling_ModePreprocessing_GYRO_Logic

double Tdet[3];
int GYRO_Threshold_Count;

///////////////BDOT Computation

int BDOT_Counter;
double Bpresent[3];
double BDOT_deltaT;
double TC_detumbling_rate_gain[3];
double TC_detumbling_bdot_gain[3];
double TC_AngMomDump_Thrsld;
double TC_momentum_dumping_gain;
double TC_comd_pitch_rate;
double DB_GyroLPF;
double DB_MagLPF;
///GYRO ext Computation

int GYRO_Counter;
int TC_Det_GYRO_Compute_Count;
double gyrodet_B[3];



///Rate reduction routine
int BDOT_Reduction_Count, BDOTNormCount, BDOT_Norm_Count;
double BDOTnorm;
double TC_BDOT_Norm_Threshold;
double TC_BDOT_Det_Thresh;


///SunAcquisition_ModePreprocessing

int SunAcquisition2ThreeAxis_autotransit;
double SunAcq_Ang_Thres;
int SunAcq3ThreeAx_trsit_cnt;
int SunAcq2ThreeAx_trsit_cnt_thres;
int SunAcq2DetMode_counter;

int ThreeAxis2DetMode_counter,f_threeaxis2safe,f_battery_safemode;

/// Timer based sunlit/eclipse

float entrytime2eclipse, orbit_time;

/// Autotransit

int TC_AutoTransitAnyMode2Det;


/// Control bytes
int CB_Detumbling_Mode;
int CB_Q_propagation;
int CB_DAD_quest;
int CB_QuestDataProcessing;
int CB_ErrorComputation;
int CB_ExtendedKalmanFilter;
int CB_LinearController;
int CB_Wheel_OverSpeed_TorqueCutOff;
int CB_DutyCycleGeneration;
int CB_AngularMomentumDumping;
int CB_SpeedBasedMomentumDumping;
int CB_Wheel_Dynamic_Friction;
int CB_Wheel_Spin_updown;
int CB_Wheel_Auto_Reconfiguration;
int CB_Two_RW_control;
int CB_Torquer_Polarity_Check;
int CB_MagFieldComp;
int CB_Sun_model;
int CB_ReferenceQuatComputation;
int CB_RefVectorGeneration;
int CB_RefRate_Computation;
int CB_Sl_Ecl_OnBrd_detection;
int CB_IMUDataProcessing;
int CB_SunSensorDataProcessing;
int CB_BDOT_Computation;
