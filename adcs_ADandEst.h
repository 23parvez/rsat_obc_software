#ifndef ADCS_ADANDEST_H_INCLUDED
#define ADCS_ADANDEST_H_INCLUDED

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


///DAD Quest
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

double Q_quest_DAD[4];

double magRoll_angle, magPitch_angle, magYaw_angle;

int wAD_updatecount;
double Qquest_update[4];
double w_BODYdeg[3];
double Q_REF_conj[4];

int I_DAD, J_DAD, K_DAD;
double NMB_NRBt[3][3], NMS_NRSt[3][3];
double Zt_Z_DAD, Zt_S_Z_DAD, Zt_Ssqr_Z_DAD;
double alpha_I_DAD[3][3], beta_S_DAD[3][3], sumX_DAD[3][3];

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
double trace_S_DAD, adj_S_DAD[3][3];


/// Qgyro

double Del_Y_theta, Del_R_theta, Del_P_theta; ///Small angle obtained by integrating Gyro data over a periof of time (mc/MC)
double Del_Q[4]; ///Delta Qs
double q_prop_out[4],Qprop_prev[4];
double Qbody[4];

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
double qk_plus[4];
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
double phi_12_temp3[3][3];
double qk_minus[4], hk_xk_minus[3];
int TC_KalmanFilter_ENABLE;
double sun_noise;
double mag_noise;
double w_prop[3], q_prop[3];

///Function Declarations
void rQ_Propagation(double q_prop[4], double w_prop[3]);
void rDAD_quest(void);
void rErrorComputation(void);
void rExtendedKalmanFilter1_p1(void);
void rExtendedKalmanFilter1_p2(void);
void rQuestDataProcessing(void);

#endif // ADCS_ADANDEST_H_INCLUDED
